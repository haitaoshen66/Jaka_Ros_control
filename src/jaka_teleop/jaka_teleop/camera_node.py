#!/usr/bin/env python3
"""
camera_node.py
作用：管理多路摄像头输入，提供统一的图像数据接口
支持USB摄像头、网络摄像头、RealSense等多种摄像头类型
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading
import time
import json
from typing import Dict, Optional, Tuple
import yaml


class CameraManager:
    """摄像头管理器"""
    
    def __init__(self):
        self.cameras = {}
        self.bridge = CvBridge()
        
    def add_camera(self, name: str, camera_config: dict) -> bool:
        """添加摄像头"""
        try:
            camera_type = camera_config.get('type', 'usb')
            
            if camera_type == 'usb':
                device_id = camera_config.get('device_id', 0)
                # 用 V4L2 后端打开，避免 GStreamer 管线报错
                cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
                if camera_config.get('use_mjpg', True):
                    # MJPG 可以减小带宽占用，常见 USB 摄像头支持
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            elif camera_type == 'ip':
                cap = cv2.VideoCapture(camera_config.get('url'))
            elif camera_type == 'realsense':
                # RealSense摄像头需要额外处理
                cap = self._setup_realsense(camera_config)
            else:
                raise ValueError(f"不支持的摄像头类型: {camera_type}")
            
            if not cap.isOpened():
                return False
                
            # 设置摄像头参数
            width = camera_config.get('width', 640)
            height = camera_config.get('height', 480)
            fps = camera_config.get('fps', 5.0)
            
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps)
            
            self.cameras[name] = {
                'capture': cap,
                'config': camera_config,
                'last_frame': None,
                'last_timestamp': 0,
                'thread': None,
                'running': False
            }
            
            return True
            
        except Exception as e:
            print(f"添加摄像头 {name} 失败: {e}")
            return False
    
    def _setup_realsense(self, config):
        """设置RealSense摄像头"""
        try:
            import pyrealsense2 as rs
            
            # 配置RealSense管道
            pipeline = rs.pipeline()
            rs_config = rs.config()
            
            # 配置彩色流
            rs_config.enable_stream(rs.stream.color, 
                                  config.get('width', 640),
                                  config.get('height', 480),
                                  rs.format.bgr8,
                                  config.get('fps', 5.0))
            
            # 配置深度流（如果需要）
            if config.get('enable_depth', False):
                rs_config.enable_stream(rs.stream.depth,
                                      config.get('depth_width', 640),
                                      config.get('depth_height', 480),
                                      rs.format.z16,
                                      config.get('fps', 5.0))
            
            pipeline.start(rs_config)
            return pipeline
            
        except ImportError:
            print("RealSense库未安装，请安装 pyrealsense2")
            return None
        except Exception as e:
            print(f"RealSense配置失败: {e}")
            return None
    
    def start_camera(self, name: str) -> bool:
        """启动摄像头"""
        if name not in self.cameras:
            return False
            
        camera = self.cameras[name]
        if camera['running']:
            return True
            
        camera['running'] = True
        camera['thread'] = threading.Thread(target=self._camera_loop, args=(name,))
        camera['thread'].daemon = True
        camera['thread'].start()
        
        return True
    
    def stop_camera(self, name: str):
        """停止摄像头"""
        if name not in self.cameras:
            return
            
        camera = self.cameras[name]
        camera['running'] = False
        
        if camera['thread']:
            camera['thread'].join(timeout=1.0)
            
    def _camera_loop(self, name: str):
        """摄像头捕获循环"""
        camera = self.cameras[name]
        capture = camera['capture']
        config = camera['config']
        
        while camera['running']:
            try:
                if config.get('type') == 'realsense':
                    frame = self._read_realsense_frame(capture, config)
                else:
                    ret, frame = capture.read()
                    if not ret:
                        continue
                
                if frame is not None:
                    # 应用图像处理
                    frame = self._process_frame(frame, config)
                    
                    camera['last_frame'] = frame.copy()
                    camera['last_timestamp'] = time.time()
                
                # 控制帧率
                time.sleep(1.0 / config.get('fps', 5.0))
                
            except Exception as e:
                print(f"摄像头 {name} 捕获错误: {e}")
                time.sleep(0.1)
    
    def _read_realsense_frame(self, pipeline, config):
        """读取RealSense帧"""
        try:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                return None
                
            # 转换为numpy数组
            color_image = np.asanyarray(color_frame.get_data())
            
            # 如果需要深度数据
            if config.get('enable_depth', False):
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    depth_image = np.asanyarray(depth_frame.get_data())
                    # 可以在这里处理深度数据
            
            return color_image
            
        except Exception as e:
            print(f"RealSense读取失败: {e}")
            return None
    
    def _process_frame(self, frame, config):
        """处理图像帧"""
        # 调整大小
        if 'resize' in config:
            width, height = config['resize']
            frame = cv2.resize(frame, (width, height))
        
        # 图像增强
        if config.get('enhance', False):
            # 对比度和亮度调整
            alpha = config.get('contrast', 1.0)
            beta = config.get('brightness', 0)
            frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
        
        # 图像滤波
        if config.get('blur', False):
            kernel_size = config.get('blur_kernel', 3)
            frame = cv2.GaussianBlur(frame, (kernel_size, kernel_size), 0)
        
        return frame
    
    def get_frame(self, name: str) -> Optional[Tuple[np.ndarray, float]]:
        """获取最新帧"""
        if name not in self.cameras:
            return None
            
        camera = self.cameras[name]
        if camera['last_frame'] is not None:
            return camera['last_frame'].copy(), camera['last_timestamp']
        
        return None
    
    def get_all_frames(self) -> Dict[str, Tuple[np.ndarray, float]]:
        """获取所有摄像头的最新帧"""
        frames = {}
        for name in self.cameras:
            frame_data = self.get_frame(name)
            if frame_data:
                frames[name] = frame_data
        return frames
    
    def list_cameras(self) -> list:
        """列出所有摄像头"""
        return list(self.cameras.keys())
    
    def remove_camera(self, name: str):
        """移除摄像头"""
        if name in self.cameras:
            self.stop_camera(name)
            camera = self.cameras[name]
            
            # 释放资源
            if hasattr(camera['capture'], 'release'):
                camera['capture'].release()
            elif hasattr(camera['capture'], 'stop'):
                camera['capture'].stop()
                
            del self.cameras[name]
    
    def cleanup(self):
        """清理所有摄像头资源"""
        for name in list(self.cameras.keys()):
            self.remove_camera(name)


class CameraNode(Node):
    """摄像头节点"""
    
    def __init__(self):
        super().__init__("camera_node")
        
        # 声明参数
        self.declare_parameter("config_file", "/home/sht/DIJA/jaka_ros2/src/jaka_teleop/config/camera_config.yaml")
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("image_quality", 80)  # JPEG压缩质量
        
        # 获取参数
        config_file = self.get_parameter("config_file").get_parameter_value().string_value
        self.publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.image_quality = self.get_parameter("image_quality").get_parameter_value().integer_value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 摄像头管理器
        self.camera_manager = CameraManager()
        
        # 发布器字典
        self.image_publishers = {}
        self.info_publishers = {}
        
        # 订阅控制命令
        self.control_sub = self.create_subscription(
            String, "/camera/control", self.control_callback, 10
        )
        
        # 状态发布器
        self.status_pub = self.create_publisher(String, "/camera/status", 10)
        
        # 加载配置
        if config_file:
            self.load_config(config_file)
        else:
            # 使用默认配置
            self.setup_default_cameras()
        
        # 发布定时器
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_images
        )
        
        # 状态发布定时器
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("✅ CameraNode 已启动")
    
    def load_config(self, config_file: str):
        """从配置文件加载摄像头配置"""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            cameras_config = config.get('cameras', {})
            
            for name, camera_config in cameras_config.items():
                if self.camera_manager.add_camera(name, camera_config):
                    self.setup_publishers(name)
                    self.camera_manager.start_camera(name)
                    self.get_logger().info(f"✅ 摄像头 {name} 配置成功")
                else:
                    self.get_logger().error(f"❌ 摄像头 {name} 配置失败")
                    
        except Exception as e:
            self.get_logger().error(f"❌ 配置文件加载失败: {e}")
            self.setup_default_cameras()
    
    def setup_default_cameras(self):
        """设置默认摄像头配置"""
        # 尝试添加USB摄像头
        default_cameras = [
            {
                'name': 'camera_0',
                'type': 'usb',
                'device_id': 0,
                'width': 640,
                'height': 480,
                'fps': 5.0
            },
            {
                'name': 'camera_1', 
                'type': 'usb',
                'device_id': 1,
                'width': 640,
                'height': 480,
                'fps': 5.0
            }
        ]
        
        for camera_config in default_cameras:
            name = camera_config['name']
            if self.camera_manager.add_camera(name, camera_config):
                self.setup_publishers(name)
                self.camera_manager.start_camera(name)
                self.get_logger().info(f"✅ 默认摄像头 {name} 启动成功")
            else:
                self.get_logger().warn(f"⚠️ 默认摄像头 {name} 启动失败")
    
    def setup_publishers(self, camera_name: str):
        """为摄像头设置发布器"""
        # 图像发布器
        image_topic = f"/camera/{camera_name}/image_raw"
        self.image_publishers[camera_name] = self.create_publisher(
            Image, image_topic, 10
        )
        
        # 摄像头信息发布器
        info_topic = f"/camera/{camera_name}/camera_info"
        self.info_publishers[camera_name] = self.create_publisher(
            CameraInfo, info_topic, 10
        )
        
        self.get_logger().info(f"📷 摄像头 {camera_name} 发布器已设置")
    
    def publish_images(self):
        """发布所有摄像头图像"""
        frames = self.camera_manager.get_all_frames()
        
        for camera_name, (frame, timestamp) in frames.items():
            if camera_name in self.image_publishers:
                try:
                    # 转换为ROS图像消息
                    image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    
                    # 设置时间戳和frame_id
                    current_time = self.get_clock().now()
                    image_msg.header.stamp = current_time.to_msg()
                    image_msg.header.frame_id = camera_name
                    
                    # 发布图像
                    self.image_publishers[camera_name].publish(image_msg)
                    
                    # 发布摄像头信息
                    self.publish_camera_info(camera_name, current_time, frame.shape)
                    
                except Exception as e:
                    self.get_logger().warn(f"发布图像失败 {camera_name}: {e}")
    
    def publish_camera_info(self, camera_name: str, timestamp, frame_shape):
        """发布摄像头信息"""
        if camera_name not in self.info_publishers:
            return
            
        try:
            camera_info = CameraInfo()
            camera_info.header.stamp = timestamp.to_msg()
            camera_info.header.frame_id = camera_name
            
            # 设置图像尺寸
            height, width = frame_shape[:2]
            camera_info.height = height
            camera_info.width = width
            
            # 简单的相机参数（实际使用时应该通过标定获得）
            fx = fy = float(width)  # 假设焦距
            cx = float(width / 2.0)
            cy = float(height / 2.0)
            
            camera_info.k = [fx, 0.0, cx,
                           0.0, fy, cy,
                           0.0, 0.0, 1.0]
            
            camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # 无畸变
            
            camera_info.r = [1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0]
            
            camera_info.p = [fx, 0.0, cx, 0.0,
                           0.0, fy, cy, 0.0,
                           0.0, 0.0, 1.0, 0.0]
            
            self.info_publishers[camera_name].publish(camera_info)
            
        except Exception as e:
            self.get_logger().warn(f"发布摄像头信息失败 {camera_name}: {e}")
    
    def publish_status(self):
        """发布摄像头状态"""
        status = {
            'timestamp': time.time(),
            'cameras': {}
        }
        
        for camera_name in self.camera_manager.list_cameras():
            frame_data = self.camera_manager.get_frame(camera_name)
            status['cameras'][camera_name] = {
                'active': frame_data is not None,
                'last_update': frame_data[1] if frame_data else 0
            }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)
    
    def control_callback(self, msg: String):
        """处理控制命令"""
        try:
            command = json.loads(msg.data)
            action = command.get('action')
            
            if action == 'add_camera':
                name = command.get('name')
                config = command.get('config')
                if name and config:
                    if self.camera_manager.add_camera(name, config):
                        self.setup_publishers(name)
                        self.camera_manager.start_camera(name)
                        self.get_logger().info(f"✅ 动态添加摄像头 {name}")
                    else:
                        self.get_logger().error(f"❌ 动态添加摄像头 {name} 失败")
            
            elif action == 'remove_camera':
                name = command.get('name')
                if name:
                    self.camera_manager.remove_camera(name)
                    # 移除发布器
                    if name in self.image_publishers:
                        del self.image_publishers[name]
                    if name in self.info_publishers:
                        del self.info_publishers[name]
                    self.get_logger().info(f"🗑️ 移除摄像头 {name}")
            
            elif action == 'list_cameras':
                cameras = self.camera_manager.list_cameras()
                self.get_logger().info(f"📋 当前摄像头: {cameras}")
                
        except json.JSONDecodeError:
            self.get_logger().warn("无法解析控制命令")
        except Exception as e:
            self.get_logger().error(f"控制命令处理失败: {e}")
    
    def destroy_node(self):
        """节点销毁时的清理"""
        self.camera_manager.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
