#!/usr/bin/env python3
"""
lerobot_recorder_node.py
作用：记录teleop数据并保存为Lerobot兼容格式
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Image, JointState
import json
import numpy as np
import h5py
import cv2
from cv_bridge import CvBridge
import threading
import time
import os
from datetime import datetime
from pathlib import Path


class LerobotRecorderNode(Node):
    def __init__(self):
        super().__init__("lerobot_recorder_node")
        
        # 参数设置
        self.declare_parameter("output_dir", "/home/sht/DIJA/jaka_ros2/src/jaka_teleop/raw_data")
        self.declare_parameter("episode_name", "episode")
        self.declare_parameter("fps", 20.0)  # 记录频率默认 20fps
        
        # 获取参数
        self.output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self.episode_name = self.get_parameter("episode_name").get_parameter_value().string_value
        self.fps = self.get_parameter("fps").get_parameter_value().double_value
        
        # 创建输出目录
        self.dataset_dir = Path(self.output_dir)
        self.dataset_dir.mkdir(parents=True, exist_ok=True)
        # 采集配置
        self.target_image_size = (640, 640)
        self.declare_parameter(
            "camera_names",
            ["front", "gripper", "camera_0", "camera_1"],
        )  # 默认同时尝试标准名与camera_node默认名
        self.video_writers = {}
        self.video_paths = {}
        self.video_frame_counts = {}
        self.current_episode_dir = None
        self.video_fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        # 记录状态
        self.is_recording = False
        self.episode_index = 0
        self.current_episode_data = []
        self.current_timestamp = 0
        
        # 数据缓存
        self.latest_oculus_transform = None
        self.latest_oculus_buttons = None
        self.latest_robot_pose = None
        self.latest_robot_joints = None
        self.latest_gripper_state = None
        self.latest_base_cmd = None
        self.latest_camera_images = {}
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # 订阅所有相关话题
        self.setup_subscriptions()
        
        # 录制控制订阅
        self.record_control_sub = self.create_subscription(
            Bool, "/lerobot/record_control", self.record_control_callback, 1
        )
        
        # 定时器，用于定期保存数据
        self.timer = self.create_timer(1.0/self.fps, self.record_data_callback)
        
        self.get_logger().info(f"✅ LerobotRecorderNode 已启动，输出目录: {self.output_dir}")

    def setup_subscriptions(self):
        """设置所有数据订阅"""
        
        # VR数据
        self.oculus_transform_sub = self.create_subscription(
            String, "/oculus/transforms", self.oculus_transform_callback, 10
        )
        self.oculus_buttons_sub = self.create_subscription(
            String, "/oculus/buttons", self.oculus_buttons_callback, 10
        )
        
        # 机器人状态
        self.robot_pose_sub = self.create_subscription(
            TwistStamped, "/jaka_driver/tool_position", self.robot_pose_callback, 10
        )
        self.robot_joints_sub = self.create_subscription(
            JointState, "/jaka_driver/joint_position", self.robot_joints_callback, 10
        )
        
        # 基座运动指令
        self.base_cmd_sub = self.create_subscription(
            Twist, "/teleop/base_cmd", self.base_cmd_callback, 10
        )
        
        # 夹爪状态（如果有对应话题）
        self.gripper_sub = self.create_subscription(
            Bool, "/teleop/gripper_cmd", self.gripper_callback, 10
        )
        
        # 摄像头图像订阅（使用camera_node提供的多路摄像头）
        self.camera_subscriptions = {}
        self.setup_camera_subscriptions()
        
        # 订阅摄像头状态
        self.camera_status_sub = self.create_subscription(
            String, "/camera/status", self.camera_status_callback, 10
        )

    def oculus_transform_callback(self, msg: String):
        """接收Oculus变换数据"""
        try:
            self.latest_oculus_transform = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("无法解析Oculus变换数据")

    def oculus_buttons_callback(self, msg: String):
        """接收Oculus按钮数据"""
        try:
            self.latest_oculus_buttons = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("无法解析Oculus按钮数据")

    def robot_pose_callback(self, msg: TwistStamped):
        """接收机器人末端位姿"""
        self.latest_robot_pose = {
            'position': [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            'orientation': [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z],
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }

    def robot_joints_callback(self, msg: JointState):
        """接收机器人关节状态"""
        self.latest_robot_joints = {
            'names': list(msg.name),
            'positions': list(msg.position),
            'velocities': list(msg.velocity) if msg.velocity else [],
            'efforts': list(msg.effort) if msg.effort else []
        }

    def base_cmd_callback(self, msg: Twist):
        """接收基座运动指令"""
        liner_x = msg.linear.x
        angular_z = msg.angular.z
        threshold = 0.5
        if abs(liner_x) < threshold :
            liner_x = 0.0
        else:
            liner_x = (liner_x - threshold) / (1 - threshold)
        if abs(angular_z) < threshold:
            angular_z = 0.0
        else :
            angular_z = (angular_z - threshold) / (1 - threshold)
        self.latest_base_cmd = {
            'linear': [liner_x],
            'angular': [angular_z]
        }
    def gripper_callback(self, msg: Bool):
        self.latest_gripper_state = msg.data
        
    def setup_camera_subscriptions(self):
        """设置摄像头订阅"""
        # 初始订阅一组预期的摄像头名称，可通过参数覆盖
        initial_cameras = (
            self.get_parameter("camera_names")
            .get_parameter_value()
            .string_array_value
        )
        for camera_name in initial_cameras:
            self.subscribe_to_camera(camera_name)
    
    def subscribe_to_camera(self, camera_name: str):
        """订阅特定摄像头"""
        if camera_name not in self.camera_subscriptions:
            topic_name = f"/camera/{camera_name}/image_raw"
            self.camera_subscriptions[camera_name] = self.create_subscription(
                Image, topic_name, 
                lambda msg, name=camera_name: self.camera_callback(msg, name), 10
            )
            self.get_logger().info(f"📷 订阅摄像头: {topic_name}")

    def camera_callback(self, msg: Image, camera_name: str):
        """接收摄像头图像"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 统一尺寸到 640x640，便于后续训练和播放
            cv_image = cv2.resize(cv_image, self.target_image_size, interpolation=cv2.INTER_LINEAR)
            self.latest_camera_images[camera_name] = {
                'image': cv_image,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'frame_id': msg.header.frame_id
            }
        except Exception as e:
            self.get_logger().warn(f"摄像头 {camera_name} 图像转换失败: {e}")

    def camera_status_callback(self, msg: String):
        """处理摄像头状态更新"""
        try:
            status = json.loads(msg.data)
            cameras = status.get('cameras', {})
            
            # 检查是否有新的摄像头
            for camera_name in cameras:
                if cameras[camera_name].get('active', False):
                    self.subscribe_to_camera(camera_name)
                    
        except json.JSONDecodeError:
            self.get_logger().warn("无法解析摄像头状态数据")

    def record_control_callback(self, msg: Bool):
        """控制录制开始/停止"""
        if msg.data and not self.is_recording:
            self.start_recording()
        elif not msg.data and self.is_recording:
            self.stop_recording()

    def start_recording(self):
        """开始录制"""
        self.close_video_writers()

        self.is_recording = True
        self.current_episode_data = []
        self.current_timestamp = 0
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_episode_name = f"{self.episode_name}_{self.episode_index:04d}_{timestamp}"
        self.current_episode_dir = self.dataset_dir / self.current_episode_name
        self.current_episode_dir.mkdir(exist_ok=True)
        self.video_writers = {}
        self.video_paths = {}
        self.video_frame_counts = {}
        
        self.get_logger().info(f"🔴 开始录制episode: {self.current_episode_name}")

    def stop_recording(self):
        """停止录制并保存数据"""
        if not self.is_recording:
            return
            
        self.is_recording = False
        
        if len(self.current_episode_data) > 0:
            self.save_episode()
            self.episode_index += 1
            self.get_logger().info(f"⏹️ 录制完成，保存了 {len(self.current_episode_data)} 帧数据")
        else:
            self.get_logger().warn("⚠️ 没有录制到数据")
            self.close_video_writers()

    def record_data_callback(self):
        """定期记录数据帧"""
        if not self.is_recording:
            return

        # 构建当前帧数据
        frame_data = {
            'timestamp': self.current_timestamp,
            'frame_index': len(self.current_episode_data),
            
            # VR输入数据
            'oculus_transform': self.latest_oculus_transform.copy() if self.latest_oculus_transform else None,
            'oculus_buttons': self.latest_oculus_buttons.copy() if self.latest_oculus_buttons else None,
            
            # 机器人状态
            'robot_pose': self.latest_robot_pose.copy() if self.latest_robot_pose else None,
            'robot_joints': self.latest_robot_joints.copy() if self.latest_robot_joints else None,
            
            # 控制指令
            'base_cmd': self.latest_base_cmd.copy() if self.latest_base_cmd else None,
            
            # 夹爪状态
            'gripper_state': self.latest_gripper_state,
            
            # 摄像头图像
            'camera_frames': {}
        }
        
        # 记录摄像头帧到视频文件，只保留轻量级元数据
        for camera_name, image_data in self.latest_camera_images.items():
            frame_data['camera_frames'][camera_name] = {
                'timestamp': image_data['timestamp'],
                'frame_id': image_data['frame_id'],
            }
            self.write_frame_to_video(camera_name, image_data['image'])
        
        self.current_episode_data.append(frame_data)
        self.current_timestamp += 1.0 / self.fps

    def write_frame_to_video(self, camera_name: str, frame):
        """将当前帧写入对应摄像头的MP4文件"""
        if self.current_episode_dir is None:
            return

        if camera_name not in self.video_writers:
            video_path = self.current_episode_dir / f"{camera_name}.mp4"
            height, width, _ = frame.shape
            writer = cv2.VideoWriter(
                str(video_path),
                self.video_fourcc,
                self.fps,
                (width, height),
            )
            if not writer.isOpened():
                self.get_logger().warn(f"无法打开视频文件用于写入: {video_path}")
                return
            self.video_writers[camera_name] = writer
            self.video_paths[camera_name] = video_path.name
            self.video_frame_counts[camera_name] = 0
            self.get_logger().info(f"🎥 开始保存摄像头 {camera_name} 视频 -> {video_path}")

        writer = self.video_writers.get(camera_name)
        if writer is not None:
            writer.write(frame)
            self.video_frame_counts[camera_name] += 1

    def close_video_writers(self):
        """释放所有视频写入器"""
        for cam, writer in list(self.video_writers.items()):
            try:
                if writer is not None:
                    writer.release()
            except Exception:
                pass
        self.video_writers = {}

    def save_episode(self):
        """保存episode到HDF5文件（仅轨迹），视频保存在同目录"""
        episode_dir = self.current_episode_dir or (self.dataset_dir / self.current_episode_name)
        episode_dir.mkdir(exist_ok=True)

        # 确保视频写入完成
        self.close_video_writers()

        # 保存HDF5格式数据
        hdf5_path = episode_dir / "data.hdf5"
        
        with h5py.File(hdf5_path, 'w') as f:
            trajectory_group = f.create_group('trajectory')
            
            # 准备数据数组
            num_frames = len(self.current_episode_data)

            # 保存轨迹数据（位置和关节）
            self.save_trajectory_data(trajectory_group, num_frames)
            
            # 保存元数据
            f.attrs['fps'] = self.fps
            f.attrs['episode_name'] = self.current_episode_name
            f.attrs['num_frames'] = num_frames
            f.attrs['created_at'] = datetime.now().isoformat()
            if self.video_paths:
                f.attrs['video_files'] = json.dumps(self.video_paths)
        
        # 保存JSON格式的元数据
        metadata = {
            'episode_name': self.current_episode_name,
            'num_frames': num_frames,
            'fps': self.fps,
            'created_at': datetime.now().isoformat(),
            'data_keys': {
                'observation_videos': self.get_observation_keys(),
                'trajectory': self.get_trajectory_keys()
            },
            'video_files': self.video_paths,
            'video_frame_counts': self.video_frame_counts,
        }
        
        with open(episode_dir / "metadata.json", 'w') as f:
            json.dump(metadata, f, indent=2)
        
        self.get_logger().info(f"💾 Episode保存至: {episode_dir}")

    def save_trajectory_data(self, traj_group, num_frames):
        """保存轨迹数据"""
        # trajectory/position: (x, y, z, orientation_x, orientation_y, orientation_z, gripper)
        position_data = []
        
        # trajectory/joint: 6个关节角度 + gripper
        joint_data = []
        
        for frame in self.current_episode_data:
            # 位置和姿态数据
            if frame['robot_pose']:
                pos = frame['robot_pose']['position']  # [x, y, z]
                ori = frame['robot_pose']['orientation']  # [rx, ry, rz]
            else:
                pos = [0.0, 0.0, 0.0]
                ori = [0.0, 0.0, 0.0]
            
            # 夹爪状态 (0.0 或 1.0)
            gripper = 1.0 if frame['gripper_state'] else 0.0
            
            # 组合: (x, y, z, orientation_x, orientation_y, orientation_z, gripper)
            position_row = pos + ori + [gripper]
            position_data.append(position_row)
            
            # 关节角度数据
            if frame['robot_joints'] and frame['robot_joints']['positions']:
                joints = frame['robot_joints']['positions'][:6]  # 取前6个关节
                # 如果关节数不足6个，用0填充
                while len(joints) < 6:
                    joints.append(0.0)
            else:
                joints = [0.0] * 6
            # 将 gripper 状态附加到关节数组末尾
            joints_with_gripper = joints + [gripper]
            
            joint_data.append(joints_with_gripper)
        
        # 保存到HDF5
        traj_group.create_dataset('position', data=np.array(position_data))
        traj_group.create_dataset('joint', data=np.array(joint_data))
        
        self.get_logger().info(f"📊 保存轨迹数据: trajectory/position - {len(position_data)} 帧 (7维)")
        self.get_logger().info(f"📊 保存轨迹数据: trajectory/joint - {len(joint_data)} 帧 (6个关节)")

    def get_observation_keys(self):
        """获取观测数据的键"""
        return list(self.video_paths.keys())

    def get_trajectory_keys(self):
        """获取轨迹数据的键"""
        return ['position', 'joint']


def main(args=None):
    rclpy.init(args=args)
    node = LerobotRecorderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.is_recording:
            node.stop_recording()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
