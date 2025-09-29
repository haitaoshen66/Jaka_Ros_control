#!/usr/bin/env python3
"""
test_camera_node.py
作用：测试camera_node功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import json
import time


class CameraTestNode(Node):
    def __init__(self):
        super().__init__('camera_test_node')
        
        self.bridge = CvBridge()
        self.received_images = {}
        
        # 订阅摄像头状态
        self.status_sub = self.create_subscription(
            String, '/camera/status', self.status_callback, 10
        )
        
        # 订阅图像（动态添加）
        self.image_subs = {}
        
        # 控制发布器
        self.control_pub = self.create_publisher(String, '/camera/control', 10)
        
        self.get_logger().info("🧪 Camera测试节点已启动")
        
        # 测试定时器
        self.test_timer = self.create_timer(5.0, self.run_tests)
        self.test_count = 0
    
    def status_callback(self, msg):
        """处理摄像头状态"""
        try:
            status = json.loads(msg.data)
            cameras = status.get('cameras', {})
            
            self.get_logger().info(f"📊 摄像头状态更新:")
            for name, info in cameras.items():
                active = "✅" if info.get('active') else "❌"
                self.get_logger().info(f"  {name}: {active}")
                
                # 为活跃的摄像头订阅图像
                if info.get('active') and name not in self.image_subs:
                    self.subscribe_to_camera(name)
                    
        except Exception as e:
            self.get_logger().warn(f"状态解析失败: {e}")
    
    def subscribe_to_camera(self, camera_name):
        """订阅摄像头图像"""
        topic = f"/camera/{camera_name}/image_raw"
        self.image_subs[camera_name] = self.create_subscription(
            Image, topic,
            lambda msg, name=camera_name: self.image_callback(msg, name), 10
        )
        self.get_logger().info(f"📷 订阅摄像头: {topic}")
    
    def image_callback(self, msg, camera_name):
        """处理图像消息"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 记录接收到的图像
            self.received_images[camera_name] = {
                'shape': cv_image.shape,
                'timestamp': time.time(),
                'ros_timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }
            
            # 显示图像信息
            height, width = cv_image.shape[:2]
            self.get_logger().info(
                f"🖼️ 收到图像 {camera_name}: {width}x{height}, "
                f"时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
            )
            
        except Exception as e:
            self.get_logger().warn(f"图像处理失败 {camera_name}: {e}")
    
    def send_control_command(self, command):
        """发送控制命令"""
        msg = String()
        msg.data = json.dumps(command)
        self.control_pub.publish(msg)
        self.get_logger().info(f"📤 发送命令: {command}")
    
    def run_tests(self):
        """运行测试"""
        self.test_count += 1
        
        if self.test_count == 1:
            # 第一次测试：列出摄像头
            self.get_logger().info("🧪 测试1: 列出摄像头")
            self.send_control_command({'action': 'list_cameras'})
            
        elif self.test_count == 2:
            # 第二次测试：添加测试摄像头
            self.get_logger().info("🧪 测试2: 添加测试摄像头")
            command = {
                'action': 'add_camera',
                'name': 'test_camera',
                'config': {
                    'type': 'usb',
                    'device_id': 2,
                    'width': 320,
                    'height': 240,
                    'fps': 15
                }
            }
            self.send_control_command(command)
            
        elif self.test_count == 3:
            # 第三次测试：检查图像接收情况
            self.get_logger().info("🧪 测试3: 检查图像接收")
            for camera_name, info in self.received_images.items():
                age = time.time() - info['timestamp']
                shape = info['shape']
                self.get_logger().info(
                    f"  {camera_name}: 形状={shape}, 延迟={age:.2f}s"
                )
            
        elif self.test_count == 4:
            # 第四次测试：移除测试摄像头
            self.get_logger().info("🧪 测试4: 移除测试摄像头")
            self.send_control_command({
                'action': 'remove_camera',
                'name': 'test_camera'
            })
            
        elif self.test_count >= 5:
            # 测试完成
            self.get_logger().info("✅ 所有测试完成")
            self.test_timer.cancel()


def main():
    rclpy.init()
    
    node = CameraTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
