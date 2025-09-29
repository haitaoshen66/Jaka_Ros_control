#!/usr/bin/env python3
# lift_node.py
# 作用：订阅升降控制指令，并通过 HTTP 接口控制电动升降腰部和头部

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import requests
import time

API_BASE_URL = "http://192.168.10.90:5000/api/extaxis"
ENABLE_URL = f"{API_BASE_URL}/enable"
MOVETO_URL = f"{API_BASE_URL}/moveto"
STATUS_URL = f"{API_BASE_URL}/status"

LIFT_STEP = 10.0  # 每次上下移动的步长（mm）
MIN_HEIGHT = 30.0
MAX_HEIGHT = 300.0

# 头部控制相关常量
HEAD_ROTATION_MIN = -90.0  # 头部左右旋转最小值（度）
HEAD_ROTATION_MAX = 90.0   # 头部左右旋转最大值（度）
HEAD_PITCH_MIN = -5.0      # 头部俯仰最小值（度，负数抬头）
HEAD_PITCH_MAX = 35.0      # 头部俯仰最大值（度，正数低头）
HEAD_ROTATION_STEP = 10.0   # 头部旋转步长（度）
HEAD_PITCH_STEP = 5.0      # 头部俯仰步长（度）


class LiftNode(Node):
    def __init__(self):
        super().__init__('lift_node')

        self.is_enabled = False
        self.current_height = 10.0  # 默认初始值
        self.current_head_rotation = 0.0  # 当前头部左右旋转角度
        self.current_head_pitch = 0.0     # 当前头部俯仰角度
        self.lift_state = "idle"

        # 启用升降系统
        self.enable_lift()

        # 查询初始状态
        self.get_current_height()
        self.get_current_head_status()

        # 订阅控制话题
        self.subscription = self.create_subscription(
            String, "/teleop/lift_cmd", self.lift_callback, 10
        )
        self.head_subscription = self.create_subscription(
            Twist, "/teleop/head_cmd", self.head_callback, 10
        )

        self.get_logger().info("✅ LiftNode 已启动")

        # 控制频率限制
        self.send_interval = 2  # 最快 20Hz
        self.last_send_time = 0

    def lift_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd not in {"up", "down", "stop"}:
            self.get_logger().warn(f"❌ 无效升降指令: {cmd}")
            return

        if cmd == "stop":
            # self.get_logger().info("🛑 收到停止指令，当前实现中忽略（无独立停止接口）")
            return

        if cmd == "up":
            self.move_to_height(299)
        elif cmd == "down":
            self.move_to_height(20)

    def head_callback(self, msg: Twist):
        """处理头部控制指令"""
        now = time.time()
        
        if now - self.last_send_time < self.send_interval:
            return  # 限频
        self.get_logger().info(f"{now}, {self.last_send_time}")
        # 处理头部左右旋转 (angular.x)
        if abs(msg.angular.x) > 0.5:  # 阈值避免抖动
            # rotation_delta = msg.angular.x * HEAD_ROTATION_STEP
            if msg.angular.x > 0:
                new_rotation = self.current_head_rotation - HEAD_ROTATION_STEP
            else:
                new_rotation = self.current_head_rotation + HEAD_ROTATION_STEP
            new_rotation = max(HEAD_ROTATION_MIN, min(HEAD_ROTATION_MAX, new_rotation))
            if new_rotation != self.current_head_rotation:
                self.current_head_rotation = new_rotation
                self.last_send_time = now
                self.move_head()
                

        # 处理头部俯仰 (angular.y)
        if abs(msg.angular.y) > 0.5:  # 阈值避免抖动
            # pitch_delta = msg.angular.y * HEAD_PITCH_STEP
            if msg.angular.y > 0:
                new_pitch = self.current_head_pitch + HEAD_PITCH_STEP
            else:
                new_pitch = self.current_head_pitch - HEAD_PITCH_STEP
            new_pitch = max(HEAD_PITCH_MIN, min(HEAD_PITCH_MAX, new_pitch))
            if new_pitch != self.current_head_pitch:
                self.current_head_pitch = new_pitch
                self.last_send_time = now
                self.move_head()    
        self.get_logger().info(f"{self.current_head_rotation:.1f}, {self.current_head_pitch:.1f}")

    def enable_lift(self):
        try:
            resp = requests.post(ENABLE_URL, json={"enable": 1}, timeout
            =1.0)
            if resp.status_code == 200:
                self.is_enabled = True
                self.get_logger().info("🔧 升降电机已使能")
            else:
                self.get_logger().warn(f"❌ 使能失败: {resp.status_code}")
        except Exception as e:
            self.get_logger().warn(f"❌ 使能请求异常: {e}")

    def get_current_height(self):
        try:
            resp = requests.get(STATUS_URL, timeout=1.0)
            if resp.status_code == 200:
                state = resp.json()
                self.get_logger().info(f"📏 当前状态: {state}")
                self.current_height = state[0]["pos"]
                self.current_head_rotation = state[2]["pos"]
                self.current_head_pitch = state[3]["pos"]
                # self.get_logger().info(f"📏 当前高度: {self.current_height:.1f} mm")
        except Exception as e:
            self.get_logger().warn(f"❌ 获取高度异常: {e}")

    def get_current_head_status(self):
        """获取当前头部状态"""
        try:
            resp = requests.get(STATUS_URL, timeout=1.0)
            if resp.status_code == 200:
                state = resp.json()
                if len(state) > 0:
                    # 假设状态返回的是 [height, 0, rotation, pitch]
                    self.current_head_rotation = state[2]["pos"]
                    self.current_head_pitch = state[3]["pos"]
                    self.get_logger().info(f"🔄 当前头部状态 - 旋转: {self.current_head_rotation:.1f}°, 俯仰: {self.current_head_pitch:.1f}°")
            else:
                self.get_logger().warn(f"❌ 查询头部状态失败: {resp.status_code}")
        except Exception as e:
            self.get_logger().warn(f"❌ 获取头部状态异常: {e}")

    def move_to_height(self, height):
        if not self.is_enabled:
            self.get_logger().warn("❌ 升降未使能")
            return
        
        target = max(MIN_HEIGHT, min(MAX_HEIGHT, height))
        try:
            move_cmd = {
                "pos": [target, 0, self.current_head_rotation, self.current_head_pitch],
                "vel": 100,
                "acc": 100
            }
            resp = requests.post(MOVETO_URL, json=move_cmd, timeout=1.0)
            if resp.status_code == 200:
                self.current_height = target
                self.get_logger().info(f"⬆️ 升降至 {target:.1f} mm")
            else:
                self.get_logger().warn(f"❌ 移动失败: {resp.status_code}")
        except Exception as e:
            self.get_logger().warn(f"❌ 移动请求异常: {e}")

    def move_head(self):
        """移动头部到指定位置"""
        if not self.is_enabled:
            self.get_logger().warn("❌ 升降未使能")
            return

        try:
            move_cmd = {
                "pos": [self.current_height, 0, self.current_head_rotation, self.current_head_pitch],
                "vel": 100,
                "acc": 100
            }
            resp = requests.post(MOVETO_URL, json=move_cmd, timeout=1.0)
            if resp.status_code == 200:
                self.get_logger().info(f"⬆️")
            else:
                self.get_logger().warn(f"❌ 移动失败: {resp.status_code}")
            
        except Exception as e:
            self.get_logger().warn(f"❌ 移动请求异常: {e}")

    def move_up(self):
        new_height = self.current_height + LIFT_STEP
        if new_height > MAX_HEIGHT:
            self.get_logger().warn("⚠️ 已达最大高度")
        else:
            self.move_to_height(new_height)

    def move_down(self):
        new_height = self.current_height - LIFT_STEP
        if new_height < MIN_HEIGHT:
            self.get_logger().warn("⚠️ 已达最小高度")
        else:
            self.move_to_height(new_height)


def main(args=None):
    rclpy.init(args=args)
    node = LiftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
