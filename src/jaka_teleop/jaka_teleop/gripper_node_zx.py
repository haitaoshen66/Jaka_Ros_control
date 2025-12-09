#!/usr/bin/env python3
# gripper_node_zx.py
# 作用：接收夹爪控制指令，使用 ZX 夹爪（Modbus/RS485）控制真实夹爪开/关

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time
import sys
sys.path.append('/home/sht/DIJA/Pr/action_sequence')
from gripper_controller import GripperController

class GripperControlNodeZX(Node):
    def __init__(self):
        super().__init__('gripper_node_zx')
        # ZX 夹爪参数（可根据实际情况修改）
        self.device = '/dev/ttyUSB0'  # 串口设备
        self.baud = 115200
        self.parity = 'N'
        self.stopbits = 1
        self.timeout = 0.3
        self.slave = 1
        self.gripper = GripperController(
            self.device, self.baud, self.parity, self.stopbits, self.timeout, self.slave
        )
        self.get_logger().info("✅ ZX夹爪控制节点已启动")
        # 订阅夹爪控制命令
        self.subscription = self.create_subscription(
            Bool,
            '/teleop/gripper_cmd',
            self.gripper_callback,
            10
        )

    def gripper_callback(self, msg: Bool):
        open_gripper = msg.data
        self.get_logger().info(f"open_gripper: {open_gripper}")
        try:
            if open_gripper:
                self.gripper.open()
                self.get_logger().info("✅ 已打开夹爪")
            else:
                self.gripper.close()
                self.get_logger().info("✅ 已关闭夹爪")
        except Exception as e:
            self.get_logger().error(f"❌ 夹爪控制异常: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNodeZX()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
