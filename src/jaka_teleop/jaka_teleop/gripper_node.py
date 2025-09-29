#!/usr/bin/env python3
# gripper_node.py
# 作用：接收夹爪控制指令，调用 ROS2 服务控制夹爪开/关

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from jaka_msgs.srv import ServoMoveEnable,ServoMove
from geometry_msgs.msg import Twist
import time

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        self.gripper_init_client = self.create_client(ServoMoveEnable, '/jaka_driver/gripper_init')
        self.gripper_control_client = self.create_client(ServoMoveEnable, '/jaka_driver/gripper_control')
        # self.servo_move_enable_client = self.create_client(ServoMoveEnable, '/jaka_driver/servo_move_enable')
        # self.servo_j_client = self.create_client(ServoMove, '/jaka_driver/servo_j')
        # 等待服务可用
        self.wait_for_services()
        
        # 初始化夹爪（非阻塞方式）
        self.init_gripper_async()
        # self.enable_servo_mode()
        # 订阅夹爪控制命令
        self.subscription = self.create_subscription(
            Bool,
            '/teleop/gripper_cmd',
            self.gripper_callback,
            10
        )
        # self.gripper_rotation_subscription = self.create_subscription(
        #     Twist,
        #     '/teleop/gripper_rotation',
        #     self.gripper_rotation_callback,
        #     10
        # )
        # self.is_rotating = False 
        self.get_logger().info("✅ GripperControlNode 已启动")

    def wait_for_services(self):
        """等待服务可用"""
        self.get_logger().info("等待夹爪服务可用...")
        while not self.gripper_init_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('夹爪初始化服务不可用，继续等待...')        
        while not self.gripper_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('夹爪控制服务不可用，继续等待...')
        # while not (self.servo_move_enable_client.wait_for_service(timeout_sec=1.0) and 
        #            self.servo_j_client.wait_for_service(timeout_sec=1.0)):
        #     if not rclpy.ok():
        #         self.get_logger().error("Interrupted while waiting for the service. Exiting.")
        #         return False
        #     self.get_logger().info("Waiting for service...")
        
        self.get_logger().info("✅ 夹爪服务已可用")

    # def enable_servo_mode(self):
    #     """启用伺服模式"""
    #     request = ServoMoveEnable.Request()
    #     request.enable = True
        
    #     future = self.servo_move_enable_client.call_async(request)
    #     rclpy.spin_until_future_complete(self, future)
        
    #     if future.result() is not None:
    #         self.get_logger().info("Servo mode enabled!")
    #         return True
    #     else:
    #         self.get_logger().error("Failed to enable servo mode")
    #         return False

    # def gripper_rotation_callback(self,msg: Twist):
    #     """执行伺服移动"""
    #     if self.is_rotating:
    #         return
        
    #     liner_x = msg.linear.x
    #     liner_x = 0.01
    #     self.is_rotating = True
    #     servo_pose = ServoMove.Request()
    #     pose = [0.0, 0.0, 0.0, 0.0, 0.0]+[liner_x]
    #     # print(f"发送夹爪旋转指令: {pose}")
    #     servo_pose.pose = pose
        
    #     for i in range(10):
    #         future = self.servo_j_client.call_async(servo_pose)
    #         # rclpy.spin_until_future_complete(self, future)
    #     # time.sleep(0.5)  # 等待0.5秒，避免过快发送
            
    #     self.is_rotating = False
    #     return True

    def init_gripper_async(self):
        """异步初始化夹爪"""
        self.get_logger().info('正在初始化夹爪...')
        request = ServoMoveEnable.Request()
        request.enable = True
        future = self.gripper_init_client.call_async(request)
        
        # 添加回调处理初始化结果（非阻塞）
        future.add_done_callback(self.handle_init_response)

    def handle_init_response(self, future):
        """处理夹爪初始化响应"""
        try:
            result = future.result()
            self.get_logger().info(f"✅ 夹爪初始化完成: {result.message}")
        except Exception as e:
            self.get_logger().error(f"❌ 夹爪初始化异常: {e}")

    def control_gripper(self, open_gripper):
        """控制夹爪开/关"""
        request = ServoMoveEnable.Request()
        request.enable = open_gripper  # True表示打开，False表示关闭
        return self.gripper_control_client.call_async(request)

    def gripper_callback(self, msg: Bool):
        """夹爪控制回调函数"""
        open_gripper = msg.data
        self.get_logger().info(f"open_gripper: {open_gripper}")
        
        # 发送控制请求（异步，不阻塞）
        future = self.control_gripper(open_gripper)
        self.get_logger().info(f"📤 发送夹爪{'打开' if open_gripper else '关闭'}请求")
        
        # 添加回调处理结果（非阻塞）
        future.add_done_callback(
            lambda f: self.handle_gripper_response(f, open_gripper)
        )

    def handle_gripper_response(self, future, open_gripper):
        """处理夹爪控制响应（在单独的线程中执行）"""
        try:
            result = future.result()
            self.get_logger().info(f"result: {result}")
            if result and result.ret:
                state_str = "打开" if open_gripper else "关闭"
                self.get_logger().info(f"✅ 已{state_str}夹爪: {result.message}")
            else:
                self.get_logger().warn(f"❌ 夹爪控制请求失败: {result.message if result else '无响应'}")
        except Exception as e:
            self.get_logger().error(f"❌ 夹爪控制异常: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
