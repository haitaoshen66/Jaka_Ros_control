#!/usr/bin/env python3
"""
visualize_hdf5.py
作用：实时可视化HDF5数据文件，显示所有摄像头视角和轨迹信息
"""

import h5py
import cv2
import numpy as np
import argparse
import sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg


class HDF5Visualizer:
    def __init__(self, hdf5_path):
        self.hdf5_path = hdf5_path
        self.data = None
        self.camera_datasets = []
        self.trajectory_data = None
        self.metadata = {}
        self.current_frame = 0
        self.total_frames = 0
        self.paused = False
        self.playback_speed = 1  # 1x speed
        
    def load_data(self):
        """加载HDF5数据"""
        print(f"📂 加载HDF5文件: {self.hdf5_path}")
        
        try:
            self.data = h5py.File(self.hdf5_path, 'r')
            
            # 打印文件结构
            print("\n📋 文件结构:")
            self._print_structure(self.data, indent=0)
            
            # 加载元数据
            if hasattr(self.data, 'attrs'):
                for key, value in self.data.attrs.items():
                    self.metadata[key] = value
                    print(f"  {key}: {value}")
            
            # 查找摄像头数据集
            if 'observation' in self.data:
                obs_group = self.data['observation']
                for key in obs_group.keys():
                    if key.startswith('camera'):
                        dataset = obs_group[key]
                        self.camera_datasets.append({
                            'name': key,
                            'dataset': dataset,
                            'shape': dataset.shape
                        })
                        print(f"📷 发现摄像头: {key}, shape: {dataset.shape}")
            
            # 加载轨迹数据
            if 'trajectory' in self.data:
                traj_group = self.data['trajectory']
                self.trajectory_data = {}
                for key in traj_group.keys():
                    self.trajectory_data[key] = traj_group[key][:]
                    print(f"📊 加载轨迹数据: {key}, shape: {traj_group[key].shape}")
            
            # 确定总帧数
            if self.camera_datasets:
                self.total_frames = self.camera_datasets[0]['shape'][0]
            elif self.trajectory_data:
                self.total_frames = list(self.trajectory_data.values())[0].shape[0]
            
            print(f"\n✅ 数据加载完成")
            print(f"📊 总帧数: {self.total_frames}")
            print(f"🎥 摄像头数量: {len(self.camera_datasets)}")
            
            if len(self.camera_datasets) == 0:
                print("\n⚠️  警告：没有找到摄像头数据！")
                print("💡 提示：请确保已经:")
                print("   1. 在launch文件中添加了 camera_names 参数")
                print("   2. 重新构建和启动系统")
                print("   3. 摄像头节点正常运行并发布图像")
                print("\n📈 将仅显示轨迹数据的图表可视化")
            
            return True
            
        except Exception as e:
            print(f"❌ 加载失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _print_structure(self, group, indent=0):
        """递归打印HDF5结构"""
        prefix = "  " * indent
        for key in group.keys():
            item = group[key]
            if isinstance(item, h5py.Group):
                print(f"{prefix}📁 {key}/")
                self._print_structure(item, indent + 1)
            else:
                print(f"{prefix}�� {key}: {item.shape} {item.dtype}")
    
    def create_trajectory_plot(self, frame_idx):
        """创建轨迹数据的图表"""
        fig = plt.figure(figsize=(12, 8))
        
        # 创建3x2的子图布局
        gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
        
        # 1. 位置曲线 (x, y, z)
        if 'position' in self.trajectory_data:
            ax1 = fig.add_subplot(gs[0, :])
            pos_data = self.trajectory_data['position']
            frames = np.arange(len(pos_data))
            
            ax1.plot(frames, pos_data[:, 0], 'r-', label='X', linewidth=2)
            ax1.plot(frames, pos_data[:, 1], 'g-', label='Y', linewidth=2)
            ax1.plot(frames, pos_data[:, 2], 'b-', label='Z', linewidth=2)
            
            # 标记当前帧
            ax1.axvline(frame_idx, color='k', linestyle='--', linewidth=2, alpha=0.7)
            ax1.scatter([frame_idx], [pos_data[frame_idx, 0]], color='r', s=100, zorder=5)
            ax1.scatter([frame_idx], [pos_data[frame_idx, 1]], color='g', s=100, zorder=5)
            ax1.scatter([frame_idx], [pos_data[frame_idx, 2]], color='b', s=100, zorder=5)
            
            ax1.set_xlabel('Frame', fontsize=12)
            ax1.set_ylabel('Position (m)', fontsize=12)
            ax1.set_title('End-effector Position', fontsize=14, fontweight='bold')
            ax1.legend(loc='upper right')
            ax1.grid(True, alpha=0.3)
        
        # 2. 姿态曲线 (rx, ry, rz)
        if 'position' in self.trajectory_data and self.trajectory_data['position'].shape[1] >= 6:
            ax2 = fig.add_subplot(gs[1, 0])
            pos_data = self.trajectory_data['position']
            frames = np.arange(len(pos_data))
            
            ax2.plot(frames, pos_data[:, 3], 'r-', label='RX', linewidth=2)
            ax2.plot(frames, pos_data[:, 4], 'g-', label='RY', linewidth=2)
            ax2.plot(frames, pos_data[:, 5], 'b-', label='RZ', linewidth=2)
            
            ax2.axvline(frame_idx, color='k', linestyle='--', linewidth=2, alpha=0.7)
            
            ax2.set_xlabel('Frame', fontsize=12)
            ax2.set_ylabel('Orientation (rad)', fontsize=12)
            ax2.set_title('End-effector Orientation', fontsize=14, fontweight='bold')
            ax2.legend(loc='upper right')
            ax2.grid(True, alpha=0.3)
        
        # 3. 夹爪状态
        if 'position' in self.trajectory_data and self.trajectory_data['position'].shape[1] >= 7:
            ax3 = fig.add_subplot(gs[1, 1])
            pos_data = self.trajectory_data['position']
            frames = np.arange(len(pos_data))
            
            ax3.plot(frames, pos_data[:, 6], 'purple', linewidth=3)
            ax3.axvline(frame_idx, color='k', linestyle='--', linewidth=2, alpha=0.7)
            ax3.scatter([frame_idx], [pos_data[frame_idx, 6]], color='purple', s=150, zorder=5)
            
            ax3.set_xlabel('Frame', fontsize=12)
            ax3.set_ylabel('Gripper State', fontsize=12)
            ax3.set_title('Gripper (0=Closed, 1=Open)', fontsize=14, fontweight='bold')
            ax3.set_ylim([-0.1, 1.1])
            ax3.grid(True, alpha=0.3)
        
        # 4. 关节角度 (前3个)
        if 'joint' in self.trajectory_data:
            ax4 = fig.add_subplot(gs[2, 0])
            joint_data = self.trajectory_data['joint']
            frames = np.arange(len(joint_data))
            
            colors = ['r', 'g', 'b']
            for i in range(min(3, joint_data.shape[1])):
                ax4.plot(frames, joint_data[:, i], colors[i], label=f'J{i+1}', linewidth=2)
            
            ax4.axvline(frame_idx, color='k', linestyle='--', linewidth=2, alpha=0.7)
            
            ax4.set_xlabel('Frame', fontsize=12)
            ax4.set_ylabel('Angle (rad)', fontsize=12)
            ax4.set_title('Joints 1-3', fontsize=14, fontweight='bold')
            ax4.legend(loc='upper right')
            ax4.grid(True, alpha=0.3)
        
        # 5. 关节角度 (后3个)
        if 'joint' in self.trajectory_data and joint_data.shape[1] >= 6:
            ax5 = fig.add_subplot(gs[2, 1])
            
            colors = ['r', 'g', 'b']
            for i in range(3, min(6, joint_data.shape[1])):
                ax5.plot(frames, joint_data[:, i], colors[i-3], label=f'J{i+1}', linewidth=2)
            
            ax5.axvline(frame_idx, color='k', linestyle='--', linewidth=2, alpha=0.7)
            
            ax5.set_xlabel('Frame', fontsize=12)
            ax5.set_ylabel('Angle (rad)', fontsize=12)
            ax5.set_title('Joints 4-6', fontsize=14, fontweight='bold')
            ax5.legend(loc='upper right')
            ax5.grid(True, alpha=0.3)
        
        # 添加总体信息
        fps = self.metadata.get('fps', 20.0)
        time_sec = frame_idx / fps
        info_text = f"Frame: {frame_idx + 1}/{self.total_frames}  |  Time: {time_sec:.2f}s  |  FPS: {fps:.1f}"
        fig.suptitle(info_text, fontsize=16, fontweight='bold', y=0.98)
        
        # 转换为OpenCV图像
        canvas = FigureCanvasAgg(fig)
        canvas.draw()
        buf = canvas.buffer_rgba()
        img = np.asarray(buf)
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        
        plt.close(fig)
        return img
    
    def create_display_frame(self, frame_idx):
        """创建显示帧，包含所有摄像头和信息overlay"""
        if not self.camera_datasets:
            # 没有摄像头数据，显示轨迹图表
            if self.trajectory_data:
                return self.create_trajectory_plot(frame_idx)
            else:
                # 完全没有数据
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "No data available", (150, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                return frame
        
        # 有摄像头数据的情况
        num_cameras = len(self.camera_datasets)
        
        if num_cameras == 1:
            img = self.camera_datasets[0]['dataset'][frame_idx]
            display = img.copy()
        elif num_cameras == 2:
            img1 = self.camera_datasets[0]['dataset'][frame_idx]
            img2 = self.camera_datasets[1]['dataset'][frame_idx]
            h, w = img1.shape[:2]
            display = np.zeros((h, w * 2, 3), dtype=np.uint8)
            display[:, :w] = img1
            display[:, w:] = img2
            
            cv2.putText(display, self.camera_datasets[0]['name'], (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display, self.camera_datasets[1]['name'], (w + 10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cols = 2
            rows = (num_cameras + 1) // 2
            img_h, img_w = self.camera_datasets[0]['dataset'][frame_idx].shape[:2]
            
            display = np.zeros((img_h * rows, img_w * cols, 3), dtype=np.uint8)
            
            for i, cam_data in enumerate(self.camera_datasets):
                row = i // cols
                col = i % cols
                img = cam_data['dataset'][frame_idx]
                
                y_start = row * img_h
                x_start = col * img_w
                display[y_start:y_start + img_h, x_start:x_start + img_w] = img
                
                cv2.putText(display, cam_data['name'], (x_start + 10, y_start + 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        display = self._add_trajectory_overlay(display, frame_idx)
        display = self._add_frame_info(display, frame_idx)
        
        return display
    
    def _add_trajectory_overlay(self, frame, frame_idx):
        """添加轨迹信息overlay"""
        if not self.trajectory_data:
            return frame
        
        overlay = frame.copy()
        h, w = frame.shape[:2]
        info_height = 180
        cv2.rectangle(overlay, (0, h - info_height), (w, h), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.7, frame, 0.3, 0)
        
        y_offset = h - info_height + 25
        line_height = 25
        
        if 'position' in self.trajectory_data:
            pos_data = self.trajectory_data['position'][frame_idx]
            text = f"Position: x={pos_data[0]:.3f} y={pos_data[1]:.3f} z={pos_data[2]:.3f}"
            cv2.putText(frame, text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            y_offset += line_height
            
            if len(pos_data) >= 6:
                text = f"Orient: rx={pos_data[3]:.3f} ry={pos_data[4]:.3f} rz={pos_data[5]:.3f}"
                cv2.putText(frame, text, (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                y_offset += line_height
            
            if len(pos_data) >= 7:
                gripper_state = "Open" if pos_data[6] > 0.5 else "Closed"
                color = (0, 255, 0) if pos_data[6] > 0.5 else (0, 0, 255)
                text = f"Gripper: {gripper_state} ({pos_data[6]:.2f})"
                cv2.putText(frame, text, (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                y_offset += line_height
        
        if 'joint' in self.trajectory_data:
            joint_data = self.trajectory_data['joint'][frame_idx]
            text = f"Joints: [{', '.join([f'{j:.2f}' for j in joint_data[:6]])}]"
            cv2.putText(frame, text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)
            y_offset += line_height
        
        return frame
    
    def _add_frame_info(self, frame, frame_idx):
        """添加帧信息"""
        h, w = frame.shape[:2]
        
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 40), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.6, frame, 0.4, 0)
        
        fps = self.metadata.get('fps', 20.0)
        time_sec = frame_idx / fps
        text = f"Frame: {frame_idx + 1}/{self.total_frames}  Time: {time_sec:.2f}s  FPS: {fps:.1f}"
        cv2.putText(frame, text, (10, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        if self.paused:
            cv2.putText(frame, "PAUSED", (w - 120, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        else:
            speed_text = f"{self.playback_speed:.1f}x"
            cv2.putText(frame, speed_text, (w - 80, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        progress_width = w - 20
        progress_height = 5
        progress_y = h - 10
        
        cv2.rectangle(frame, (10, progress_y - progress_height), 
                     (10 + progress_width, progress_y), (100, 100, 100), -1)
        
        progress = int((frame_idx / max(self.total_frames - 1, 1)) * progress_width)
        cv2.rectangle(frame, (10, progress_y - progress_height), 
                     (10 + progress, progress_y), (0, 255, 0), -1)
        
        return frame
    
    def run(self):
        """运行可视化"""
        if not self.load_data():
            return
        
        if self.total_frames == 0:
            print("❌ 没有可显示的帧")
            return
        
        window_name = f"HDF5 Visualizer - {Path(self.hdf5_path).name}"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        print("\n🎮 控制说明:")
        print("  空格键: 暂停/继续")
        print("  ←/→: 上一帧/下一帧 (暂停时)")
        print("  ↑/↓: 加速/减速")
        print("  R: 重新开始")
        print("  Q/ESC: 退出")
        print("  S: 保存当前帧截图")
        
        fps = self.metadata.get('fps', 20.0)
        frame_delay = int(1000 / fps)
        
        try:
            while True:
                if not self.paused:
                    display_frame = self.create_display_frame(self.current_frame)
                    cv2.imshow(window_name, display_frame)
                    
                    self.current_frame += 1
                    if self.current_frame >= self.total_frames:
                        self.current_frame = 0
                    
                    wait_time = max(1, int(frame_delay / self.playback_speed))
                else:
                    display_frame = self.create_display_frame(self.current_frame)
                    cv2.imshow(window_name, display_frame)
                    wait_time = 30
                
                key = cv2.waitKey(wait_time) & 0xFF
                
                if key == ord('q') or key == 27:
                    break
                elif key == ord(' '):
                    self.paused = not self.paused
                    print(f"{'⏸ 暂停' if self.paused else '▶ 播放'}")
                elif key == ord('r'):
                    self.current_frame = 0
                    print("🔄 重新开始")
                elif key == 83 or key == ord('d'):
                    if self.paused:
                        self.current_frame = min(self.current_frame + 1, self.total_frames - 1)
                elif key == 81 or key == ord('a'):
                    if self.paused:
                        self.current_frame = max(self.current_frame - 1, 0)
                elif key == 82 or key == ord('w'):
                    self.playback_speed = min(self.playback_speed * 1.5, 8.0)
                    print(f"⏩ 速度: {self.playback_speed:.1f}x")
                elif key == 84 or key == ord('s'):
                    if key == ord('s'):
                        screenshot_path = f"frame_{self.current_frame:04d}.png"
                        cv2.imwrite(screenshot_path, display_frame)
                        print(f"💾 保存截图: {screenshot_path}")
                    else:
                        self.playback_speed = max(self.playback_speed / 1.5, 0.25)
                        print(f"⏪ 速度: {self.playback_speed:.1f}x")
        
        except KeyboardInterrupt:
            print("\n👋 用户中断")
        finally:
            cv2.destroyAllWindows()
            if self.data:
                self.data.close()
            print("✅ 可视化结束")


def main():
    parser = argparse.ArgumentParser(description='可视化HDF5数据文件')
    parser.add_argument('hdf5_file', help='HDF5文件路径')
    
    args = parser.parse_args()
    
    hdf5_path = Path(args.hdf5_file)
    if not hdf5_path.exists():
        print(f"❌ 文件不存在: {hdf5_path}")
        sys.exit(1)
    
    visualizer = HDF5Visualizer(hdf5_path)
    visualizer.run()


if __name__ == "__main__":
    main()
