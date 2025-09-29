#!/usr/bin/env python3
"""
camera_controller.py
作用：摄像头控制工具，用于动态管理摄像头
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import argparse


class CameraController:
    def __init__(self):
        rclpy.init()
        self.node = Node('camera_controller_temp')
        
        # 发布控制命令
        self.control_pub = self.node.create_publisher(String, '/camera/control', 10)
        
        # 订阅状态
        self.status_sub = self.node.create_subscription(
            String, '/camera/status', self.status_callback, 10
        )
        
        self.latest_status = None
    
    def status_callback(self, msg):
        """接收摄像头状态"""
        try:
            self.latest_status = json.loads(msg.data)
        except:
            pass
    
    def send_command(self, command):
        """发送控制命令"""
        msg = String()
        msg.data = json.dumps(command)
        self.control_pub.publish(msg)
        print(f"发送命令: {command}")
    
    def add_camera(self, name, camera_type='usb', device_id=0, width=640, height=480, fps=5):
        """添加摄像头"""
        command = {
            'action': 'add_camera',
            'name': name,
            'config': {
                'type': camera_type,
                'device_id': device_id,
                'width': width,
                'height': height,
                'fps': fps
            }
        }
        self.send_command(command)
    
    def remove_camera(self, name):
        """移除摄像头"""
        command = {
            'action': 'remove_camera',
            'name': name
        }
        self.send_command(command)
    
    def list_cameras(self):
        """列出摄像头"""
        command = {'action': 'list_cameras'}
        self.send_command(command)
    
    def get_status(self):
        """获取状态"""
        # 等待状态更新
        import time
        time.sleep(0.5)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        return self.latest_status
    
    def cleanup(self):
        """清理"""
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='摄像头控制工具')
    parser.add_argument('action', choices=['add', 'remove', 'list', 'status'],
                       help='操作类型')
    parser.add_argument('--name', help='摄像头名称')
    parser.add_argument('--type', default='usb', choices=['usb', 'ip', 'realsense'],
                       help='摄像头类型')
    parser.add_argument('--device-id', type=int, default=0, help='设备ID')
    parser.add_argument('--width', type=int, default=640, help='图像宽度')
    parser.add_argument('--height', type=int, default=480, help='图像高度')
    parser.add_argument('--fps', type=int, default=30, help='帧率')
    
    args = parser.parse_args()
    
    controller = CameraController()
    
    try:
        if args.action == 'add':
            if not args.name:
                print("错误：添加摄像头需要指定名称 (--name)")
                return
            controller.add_camera(args.name, args.type, args.device_id,
                                args.width, args.height, args.fps)
            
        elif args.action == 'remove':
            if not args.name:
                print("错误：移除摄像头需要指定名称 (--name)")
                return
            controller.remove_camera(args.name)
            
        elif args.action == 'list':
            controller.list_cameras()
            
        elif args.action == 'status':
            status = controller.get_status()
            if status:
                print("摄像头状态:")
                for name, info in status.get('cameras', {}).items():
                    active = "✅ 活跃" if info.get('active') else "❌ 非活跃"
                    print(f"  {name}: {active}")
            else:
                print("无法获取摄像头状态")
                
    except KeyboardInterrupt:
        pass
    finally:
        controller.cleanup()


if __name__ == '__main__':
    main()
