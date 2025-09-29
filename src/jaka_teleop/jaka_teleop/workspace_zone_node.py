#!/usr/bin/env python3
# workspace_zone_node.py
# 作用：订阅 workspace 区域指令，调用关节空间服务移动机械臂到预设姿态

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from jaka_msgs.srv import Move


class WorkspaceZoneNode(Node):
    def __init__(self):
        super().__init__("workspace_zone_node")

        # 订阅 workspace 区域话题
        self.subscription = self.create_subscription(
            String, "/teleop/workspace_zone", self.zone_callback, 10
        )
        
        self.is_running = False
        
        # 服务客户端
        self.move_client = self.create_client(Move, "/jaka_driver/joint_move")
        
        # 使用异步方式等待服务
        self.service_ready = False
        self.check_service_timer = self.create_timer(0.5, self.check_service)

        # 预设工作区间的关节角（单位: rad）
        self.zone_poses = {
            "high": [1.8,-0.763,-1.176,-0.159,-1.0,-0.57], # 4层
            "mid": [1.4,-0.763,-1.176,-0.159,-1.0,-0.87], # 3层
            "low": [1.3,-0.663,-1.176,-0.159,-1.7,-0.87], # 放药
        }

        self.get_logger().info("✅ WorkspaceZoneNode (Joint Mode) 已启动")

    def check_service(self):
        """检查服务是否可用"""
        if self.move_client.service_is_ready():
            self.service_ready = True
            self.check_service_timer.cancel()
            self.get_logger().info("✅ JointMove 服务已就绪")
        else:
            self.get_logger().info("等待 JointMove 服务...")

    def zone_callback(self, msg: String):
        """处理工作区切换请求"""
        if not self.service_ready:
            self.get_logger().warn("❌ JointMove 服务尚未就绪，忽略指令")
            return

        if self.is_running:
            self.get_logger().warn("⚠️ 正在执行上一条移动指令，忽略本次请求")
            return

        zone_name = msg.data.strip().lower()
        if zone_name not in self.zone_poses:
            self.get_logger().warn(f"❌ 未识别的工作区: {zone_name}")
            return

        self.is_running = True
        joint_target = self.zone_poses[zone_name]
        self.get_logger().info(f"📍 正在移动到 {zone_name}: {joint_target}")

        # 创建请求
        request = Move.Request()
        request.pose = joint_target
        request.has_ref = False
        request.ref_joint = [0.0]  # 浮点数类型
        request.mvvelo = 5.0
        request.mvacc = 5.0
        request.mvtime = 0.0
        request.mvradii = 0.0
        request.coord_mode = 0
        request.index = 0

        # 异步调用服务
        future = self.move_client.call_async(request)
        future.add_done_callback(lambda f: self.move_callback(f, zone_name))

    def move_callback(self, future, zone_name):
        """处理移动完成回调"""
        try:
            result = future.result()
            if result and result.ret:
                self.get_logger().info(f"✅ 成功移动到 {zone_name}: {result.message}")
            else:
                self.get_logger().warn(f"❌ 移动失败: {zone_name} - {result.message if result else '无响应'}")
        except Exception as e:
            self.get_logger().error(f"❌ 移动异常: {zone_name} - {e}")
        finally:
            # 释放运行状态
            self.is_running = False


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceZoneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
