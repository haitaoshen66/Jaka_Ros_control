#!/usr/bin/env python3
"""
lerobot_control_node.py
作用：提供录制控制接口，可以通过按钮或服务控制数据录制
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
import json


class LerobotControlNode(Node):
    def __init__(self):
        super().__init__("lerobot_control_node")
        
        # 录制状态
        self.is_recording = False
        
        # 发布录制控制指令
        self.record_control_pub = self.create_publisher(
            Bool, "/lerobot/record_control", 1
        )
        
        # 订阅按钮事件（用于开始/停止录制）
        self.button_sub = self.create_subscription(
            String, "/oculus/buttons", self.button_callback, 10
        )
        
        # 创建服务接口
        self.record_service = self.create_service(
            SetBool, "/lerobot/toggle_recording", self.toggle_recording_callback
        )
        
        # 状态发布定时器
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("✅ LerobotControlNode 已启动")
        self.get_logger().info("📝 使用服务调用控制录制: ros2 service call /lerobot/toggle_recording std_srvs/srv/SetBool \"{data: true}\"")
        self.get_logger().info("🎮 或使用VR按钮 X 开始/停止录制")

    def button_callback(self, msg: String):
        """监听VR按钮，X键控制录制"""
        try:
            buttons = json.loads(msg.data)
            # 检查X按钮是否被按下（边缘触发）
            if buttons.get('X', False) and not getattr(self, '_prev_x_pressed', False):
                self.toggle_recording()

            self._prev_x_pressed = buttons.get('X', False)

        except json.JSONDecodeError:
            pass

    def toggle_recording_callback(self, request, response):
        """服务接口：开始/停止录制"""
        if request.data:
            success = self.start_recording()
            response.message = "录制已开始" if success else "录制启动失败"
        else:
            success = self.stop_recording()
            response.message = "录制已停止" if success else "录制停止失败"
        
        response.success = success
        return response

    def toggle_recording(self):
        """切换录制状态"""
        if self.is_recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        """开始录制"""
        if self.is_recording:
            self.get_logger().warn("⚠️ 已经在录制中")
            return False
        
        self.is_recording = True
        
        # 发布录制开始信号
        msg = Bool()
        msg.data = True
        self.record_control_pub.publish(msg)
        
        self.get_logger().info("🔴 开始录制数据")
        return True

    def stop_recording(self):
        """停止录制"""
        if not self.is_recording:
            self.get_logger().warn("⚠️ 当前没有在录制")
            return False
        
        self.is_recording = False
        
        # 发布录制停止信号
        msg = Bool()
        msg.data = False
        self.record_control_pub.publish(msg)
        
        self.get_logger().info("⏹️ 停止录制数据")
        return True

    def publish_status(self):
        """定期发布录制状态"""
        status = "🔴 录制中" if self.is_recording else "⚪ 待机中"
        # 不要太频繁打印状态，只在状态改变时打印
        if not hasattr(self, '_last_status') or self._last_status != self.is_recording:
            self.get_logger().info(f"📊 录制状态: {status}")
            self._last_status = self.is_recording


def main(args=None):
    rclpy.init(args=args)
    node = LerobotControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
