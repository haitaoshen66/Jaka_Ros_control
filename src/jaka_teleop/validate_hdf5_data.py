#!/usr/bin/env python3
"""
validate_hdf5_data.py
作用：验证HDF5数据文件，将多个摄像头视角合并成一个视频
"""

import h5py
import cv2
import numpy as np
import argparse
import os
from pathlib import Path
import json


class HDF5VideoValidator:
    def __init__(self, hdf5_path, output_video_path=None):
        self.hdf5_path = Path(hdf5_path)
        self.output_video_path = output_video_path or (self.hdf5_path.parent / "merged_video.mp4")
        
        # 视频参数
        self.fps = 30.0
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        
        print(f"📁 读取HDF5文件: {self.hdf5_path}")
        print(f"🎬 输出视频: {self.output_video_path}")
    
    def load_hdf5_data(self):
        """加载HDF5数据"""
        try:
            with h5py.File(self.hdf5_path, 'r') as f:
                print("\n📊 HDF5文件结构:")
                self._print_hdf5_structure(f, indent=0)
                
                # 加载观测数据（摄像头图像）
                observation_data = {}
                if 'observation' in f:
                    obs_group = f['observation']
                    for key in obs_group.keys():
                        if key.startswith('camera'):
                            data = obs_group[key][:]
                            observation_data[key] = data
                            print(f"📷 加载 {key}: {data.shape}")
                
                # 加载轨迹数据
                trajectory_data = {}
                if 'trajectory' in f:
                    traj_group = f['trajectory']
                    for key in traj_group.keys():
                        data = traj_group[key][:]
                        trajectory_data[key] = data
                        print(f"📈 加载 trajectory/{key}: {data.shape}")
                
                # 加载元数据
                metadata = {}
                for attr_name in f.attrs.keys():
                    metadata[attr_name] = f.attrs[attr_name]
                    print(f"ℹ️  {attr_name}: {metadata[attr_name]}")
                
                return observation_data, trajectory_data, metadata
                
        except Exception as e:
            print(f"❌ 加载HDF5文件失败: {e}")
            return None, None, None
    
    def _print_hdf5_structure(self, group, indent=0):
        """递归打印HDF5结构"""
        prefix = "  " * indent
        for key in group.keys():
            item = group[key]
            if isinstance(item, h5py.Group):
                print(f"{prefix}📁 {key}/")
                self._print_hdf5_structure(item, indent + 1)
            else:
                print(f"{prefix}📄 {key}: {item.shape} {item.dtype}")
    
    def create_merged_video(self, observation_data, trajectory_data, metadata):
        """创建合并视频"""
        if not observation_data:
            print("❌ 没有找到摄像头数据")
            return False
        
        # 获取摄像头数据
        camera_keys = sorted([k for k in observation_data.keys() if k.startswith('camera')])
        print(f"📷 找到摄像头: {camera_keys}")
        
        if len(camera_keys) == 0:
            print("❌ 没有摄像头数据")
            return False
        
        # 获取视频参数
        first_camera = observation_data[camera_keys[0]]
        num_frames, height, width = first_camera.shape[:3]
        
        # 确定合并布局
        if len(camera_keys) == 1:
            # 单摄像头，直接使用原图
            merged_width = width
            merged_height = height
            layout = "single"
        elif len(camera_keys) == 2:
            # 双摄像头，左右排列
            merged_width = width * 2
            merged_height = height
            layout = "horizontal"
        elif len(camera_keys) <= 4:
            # 多摄像头，2x2布局
            merged_width = width * 2
            merged_height = height * 2
            layout = "grid"
        else:
            # 更多摄像头，只取前4个
            camera_keys = camera_keys[:4]
            merged_width = width * 2
            merged_height = height * 2
            layout = "grid"
        
        print(f"🎬 视频参数: {num_frames}帧, {merged_width}x{merged_height}, 布局: {layout}")
        
        # 获取FPS
        fps = metadata.get('fps', 30.0)
        
        # 创建视频写入器
        video_writer = cv2.VideoWriter(
            str(self.output_video_path), 
            self.fourcc, 
            fps, 
            (merged_width, merged_height)
        )
        
        if not video_writer.isOpened():
            print("❌ 无法创建视频文件")
            return False
        
        print(f"🎥 开始生成视频，共{num_frames}帧...")
        
        # 生成每一帧
        for frame_idx in range(num_frames):
            merged_frame = self._create_merged_frame(
                observation_data, camera_keys, frame_idx, 
                merged_width, merged_height, layout
            )
            
            # 添加轨迹信息overlay
            if trajectory_data:
                merged_frame = self._add_trajectory_overlay(
                    merged_frame, trajectory_data, frame_idx
                )
            
            # 添加帧信息
            merged_frame = self._add_frame_info(
                merged_frame, frame_idx, num_frames
            )
            
            # 写入视频
            video_writer.write(merged_frame)
            
            # 显示进度
            if (frame_idx + 1) % 10 == 0 or frame_idx == num_frames - 1:
                progress = (frame_idx + 1) / num_frames * 100
                print(f"📊 进度: {frame_idx + 1}/{num_frames} ({progress:.1f}%)")
        
        video_writer.release()
        print(f"✅ 视频生成完成: {self.output_video_path}")
        return True
    
    def _create_merged_frame(self, observation_data, camera_keys, frame_idx, 
                           merged_width, merged_height, layout):
        """创建合并帧"""
        merged_frame = np.zeros((merged_height, merged_width, 3), dtype=np.uint8)
        
        if layout == "single":
            # 单摄像头
            img = observation_data[camera_keys[0]][frame_idx]
            merged_frame = cv2.resize(img, (merged_width, merged_height))
            
        elif layout == "horizontal":
            # 水平排列（左右）
            img1 = observation_data[camera_keys[0]][frame_idx]
            img2 = observation_data[camera_keys[1]][frame_idx]
            
            h, w = img1.shape[:2]
            img1_resized = cv2.resize(img1, (w, h))
            img2_resized = cv2.resize(img2, (w, h))
            
            merged_frame[:, :w] = img1_resized
            merged_frame[:, w:] = img2_resized
            
        elif layout == "grid":
            # 2x2网格布局
            h = merged_height // 2
            w = merged_width // 2
            
            for i, camera_key in enumerate(camera_keys[:4]):
                img = observation_data[camera_key][frame_idx]
                img_resized = cv2.resize(img, (w, h))
                
                row = i // 2
                col = i % 2
                y_start = row * h
                y_end = (row + 1) * h
                x_start = col * w
                x_end = (col + 1) * w
                
                merged_frame[y_start:y_end, x_start:x_end] = img_resized
        
        return merged_frame
    
    def _add_trajectory_overlay(self, frame, trajectory_data, frame_idx):
        """添加轨迹信息overlay"""
        overlay = frame.copy()
        
        # 在左上角显示轨迹信息
        y_pos = 30
        
        if 'position' in trajectory_data and frame_idx < len(trajectory_data['position']):
            pos_data = trajectory_data['position'][frame_idx]
            if len(pos_data) >= 7:
                x, y, z = pos_data[:3]
                rx, ry, rz = pos_data[3:6]
                gripper = pos_data[6]
                
                cv2.putText(overlay, f"Position: ({x:.3f}, {y:.3f}, {z:.3f})", 
                           (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                y_pos += 25
                
                cv2.putText(overlay, f"Rotation: ({rx:.3f}, {ry:.3f}, {rz:.3f})", 
                           (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                y_pos += 25
                
                gripper_status = "OPEN" if gripper < 0.5 else "CLOSE"
                cv2.putText(overlay, f"Gripper: {gripper_status}", 
                           (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                y_pos += 25
        
        if 'joint' in trajectory_data and frame_idx < len(trajectory_data['joint']):
            joint_data = trajectory_data['joint'][frame_idx]
            joint_str = ", ".join([f"{j:.2f}" for j in joint_data])
            cv2.putText(overlay, f"Joints: [{joint_str}]", 
                       (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return overlay
    
    def _add_frame_info(self, frame, frame_idx, total_frames):
        """添加帧信息"""
        # 在右上角显示帧信息
        frame_text = f"Frame: {frame_idx + 1}/{total_frames}"
        text_size = cv2.getTextSize(frame_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
        x_pos = frame.shape[1] - text_size[0] - 10
        
        cv2.putText(frame, frame_text, (x_pos, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return frame
    
    def validate_and_create_video(self):
        """验证数据并创建视频"""
        print("🔍 开始验证HDF5数据...")
        
        # 加载数据
        observation_data, trajectory_data, metadata = self.load_hdf5_data()
        
        if observation_data is None:
            return False
        
        # 创建视频
        success = self.create_merged_video(observation_data, trajectory_data, metadata)
        
        if success:
            print(f"\n✅ 验证完成！视频已保存到: {self.output_video_path}")
            print(f"📺 使用以下命令播放视频:")
            print(f"   vlc {self.output_video_path}")
            print(f"   或")
            print(f"   ffplay {self.output_video_path}")
        
        return success


def main():
    parser = argparse.ArgumentParser(description="验证HDF5数据并生成合并视频")
    parser.add_argument("hdf5_path", help="HDF5数据文件路径")
    parser.add_argument("-o", "--output", help="输出视频路径", default=None)
    parser.add_argument("--fps", type=float, default=30.0, help="输出视频帧率")
    
    args = parser.parse_args()
    
    # 检查输入文件
    if not os.path.exists(args.hdf5_path):
        print(f"❌ 文件不存在: {args.hdf5_path}")
        return
    
    # 创建验证器
    validator = HDF5VideoValidator(args.hdf5_path, args.output)
    validator.fps = args.fps
    
    # 执行验证
    validator.validate_and_create_video()


if __name__ == "__main__":
    main()
