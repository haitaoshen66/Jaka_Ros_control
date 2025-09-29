#!/usr/bin/env python3
# base_motion_node.py
# 作用：订阅摇杆控制指令，并通过 TCP + JSON 请求控制底盘

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import json
import time


class BaseMotionNode(Node):
    def __init__(self):
        super().__init__("base_motion_node")

        # 底盘控制器配置
        self.host = "192.168.10.10"
        self.port = 31001
        self.command_prefix = "/api/joy_control"

        # 控制频率限制
        self.send_interval = 0.03  # 最快 20Hz
        self.last_send_time = 0

        # 最大速度限制
        self.max_linear = 1  # m/s
        self.max_angular = 1  # rad/s

        # 控制倍率参数
        # self.declare_parameter("control_scale", 1)

        # 订阅控制指令话题
        self.subscription = self.create_subscription(
            Twist, "/teleop/base_cmd", self.cmd_callback, 10
        )

        self.get_logger().info("✅ BaseMotionNode 已启动")

    def cmd_callback(self, msg: Twist):
        now = time.time()
        if now - self.last_send_time < self.send_interval:
            return  # 限频

        # scale = self.get_parameter("control_scale").get_parameter_value().double_value
        # if scale < 0.5:
        scale = 1.0
        # self.get_logger().info(f"当前控制比例: {scale}")
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

        vx = max(min(liner_x * 5 * scale, self.max_linear), -self.max_linear)
        wz = max(min(angular_z * 5 * scale, self.max_angular), -self.max_angular)

        # 构造 URL 风格控制命令（与主控一致）
        param_str = f"angular_velocity={wz:.3f}&linear_velocity={vx:.3f}"
        full_command = f"{self.command_prefix}?{param_str}&uuid=123456"

        try:
            response = self.send_tcp_command(full_command)
            # self.get_logger().info(f"📤 指令: {full_command}")
            # self.get_logger().info(f"📥 响应: {json.dumps(response)}")
            self.last_send_time = now
        except Exception as e:
            self.get_logger().warn(f"❌ 发送失败: {e}")

    def send_tcp_command(self, command: str):
        """发送 TCP 命令并接收 JSON 响应"""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((self.host, self.port))
            sock.sendall(command.encode("utf-8"))
            response = sock.recv(4096).decode("utf-8")
            return json.loads(response)


def main(args=None):
    rclpy.init(args=args)
    node = BaseMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
