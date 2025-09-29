#!/usr/bin/env python3
# buttons_node.py
# 作用：解析 Oculus 手柄按钮消息，发布对应的控制指令（话题/参数）

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterEvent
import json


class ButtonsNode(Node):
    def __init__(self):
        super().__init__("buttons_node")

        # 订阅按钮输入
        self.subscription = self.create_subscription(
            String, "/oculus/buttons", self.listener_callback, 10
        )

        # 离散状态控制
        self.reset_pub = self.create_publisher(Bool, "/teleop/reset_pose", 1)
        self.workspace_pub = self.create_publisher(String, "/teleop/workspace_zone", 10)
        # self.gripper_rotation = self.create_publisher(Twist, "/teleop/gripper_rotation", 10)
        # 连续控制（动作）
        self.base_pub = self.create_publisher(Twist, "/teleop/base_cmd", 10)
        self.gripper_pub = self.create_publisher(Bool, "/teleop/gripper_cmd", 10)
        self.lift_pub = self.create_publisher(String, "/teleop/lift_cmd", 10)
        self.enable_pub = self.create_publisher(Bool, "/teleop/enable", 10)
        # 头部旋转
        self.head_pub = self.create_publisher(Twist, "/teleop/head_cmd", 10)
        # 控制比例（快慢切换）
        # self.declare_parameter("control_scale", 1.0)  # 默认1.0（快）
        self.reseting = False
        self.last_reset_button = False
        self.last_mode_button = False
        # 状态缓存
        self.last_gripper_state = None  # 避免重复发
        self.last_buttons = {}

    def listener_callback(self, msg):
        try:
            buttons = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("按钮JSON解析失败")
            return

        self.last_buttons = buttons

        # === 右手离散功能 ===
        cur_reset_button = buttons.get("B")
        if not buttons.get("A"):
            if cur_reset_button != self.last_reset_button:
                self.last_reset_button = cur_reset_button
                if cur_reset_button and not self.reseting:
                    self.get_logger().info("🔄 重置机器人末端位姿")
                    self.reseting = True
                    self.reset_pub.publish(Bool(data=True))
                elif cur_reset_button and self.reseting:
                    self.get_logger().info("✅ 结束重置模式")
                    self.reseting = False
                    self.reset_pub.publish(Bool(data=False))

        if buttons.get("A") and buttons.get("Y"):
            # self.get_logger().info("切换工作区间: 高")
            self.reseting = True
            self.reset_pub.publish(Bool(data=True))
            self.workspace_pub.publish(String(data="high"))

        if buttons.get("A") and buttons.get("B"):
            # self.get_logger().info("切换工作区间: 中")
            self.reseting = True
            self.reset_pub.publish(Bool(data=True))
            self.workspace_pub.publish(String(data="mid"))

        if buttons.get("A") and buttons.get("X"):
            # self.get_logger().info("切换工作区间: 低")
            self.reseting = True
            self.reset_pub.publish(Bool(data=True))
            self.workspace_pub.publish(String(data="low"))

        # === 左手离散功能 ===
        # cur_mode_button = buttons.get("X")
        # if not buttons.get("A"):
            # if cur_mode_button != self.last_mode_button and cur_mode_button:
            #     new_scale = (
            #         0.5
            #         if self.get_parameter("control_scale")
            #         .get_parameter_value()
            #         .double_value
            #         > 0.9
            #         else 1.0
            #     )
            #     self.set_parameters(
            #         [
            #             rclpy.parameter.Parameter(
            #                 name="control_scale",
            #                 value=new_scale,
            #             )
            #         ]
            #     )
            #     self.get_logger().info(f"切换控制比例为 {new_scale}")
            # self.last_mode_button = cur_mode_button

        if buttons.get("Y") and not buttons.get("A"):
            self.reseting = True
            self.reset_pub.publish(Bool(data=True))
            self.enable_pub.publish(Bool(data=True))
            self.get_logger().info("✅ 使能已发布")

        # === 连续值控制：右手 Joystick 控制底盘 ===
        if "leftJS" in buttons :
            x, y = buttons["leftJS"]
            twist = Twist()
            twist.linear.x = y  # 前后
            twist.angular.z = -x  # 左右转（右为负）
            self.base_pub.publish(twist)

        # if "leftJS" in buttons and buttons.get("A"):
        #     x, y = buttons["leftJS"]
        #     if abs(x) > 0.5:
        #         twist = Twist()
        #         twist.linear.x = -x
        #         self.gripper_rotation.publish(twist)  # 夹爪旋转（右为负）
        # === 头部旋转 ===
        # if "leftJS" in buttons:
        #     x, y = buttons["leftJS"]
        #     twist = Twist()
            # if abs(x) > abs(y):
            #     twist.angular.y = 0.0  # 上下转（上为负）
            #     twist.angular.x = x  # 左右转（左为负）
            #     self.head_pub.publish(twist)
            # elif abs(y) > abs(x):
            #     twist.angular.y = y  # 上下转（上为负）
            #     twist.angular.x = 0.0  # 左右转（左为负）
            #     self.head_pub.publish(twist)

        # === Trigger 控制夹爪 ===
        trigger_value = buttons.get("rightTrig", [0.0])[0]
        # self.get_logger().info(f"trigger_value: {trigger_value}")
        if trigger_value > 0.5:
            if self.last_gripper_state is not False:
                # self.get_logger().info("send gripper close")
                self.last_gripper_state = False
                self.gripper_pub.publish(Bool(data=False))  # 闭合
        elif trigger_value < 0.5 and trigger_value > 0.0:
            if self.last_gripper_state is not True:
                # self.get_logger().info("send gripper open")
                self.last_gripper_state = True
                self.gripper_pub.publish(Bool(data=True))  # 打开

        # === 升降控制 ===
        # if buttons.get("leftTrig", [0.0])[0] > 0.5:
        #     self.lift_pub.publish(String(data="up"))
        # elif buttons.get("leftGrip", [0.0])[0] > 0.5:
        #     self.lift_pub.publish(String(data="down"))


def main(args=None):
    rclpy.init(args=args)
    node = ButtonsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
