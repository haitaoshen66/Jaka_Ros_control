#!/usr/bin/env python3
# servo_control_node.py
# 作用：订阅 Oculus transform 并控制机器人末端增量运动，支持比例缩放与 Reset 模式

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import ParameterEvent
from jaka_msgs.srv import ServoMove, ServoMoveEnable
import json
import numpy as np
import math
from geometry_msgs.msg import TwistStamped

class JakaServoClientAsync(Node):

    def __init__(self):
        super().__init__('jaka_servo_p_clientpy')
        self.servo_move_enable_client  = self.create_client(ServoMoveEnable, '/jaka_driver/servo_move_enable')
        self.servo_p_client = self.create_client(ServoMove, '/jaka_driver/servo_p')

        while not self.servo_move_enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ServoMove.Request()

    def enable_servo_mode(self):
        enable_request = ServoMoveEnable.Request()
        enable_request.enable = True
        return self.servo_move_enable_client.call_async(enable_request)

    def send_request(self,pose_delta):# pose_delta[]
        self.req.pose = pose_delta
        return self.servo_p_client.call_async(self.req)

class ServoControlNode(Node):
    def __init__(self):
        super().__init__("servo_control_node")

        # 参数设置
        # self.declare_parameter("control_scale", 1.0)
        self.scale = 1
        self.reset_sub=self.create_subscription(Bool, "/teleop/reset_pose", self.reset_callback, 1)
        self.tool_pos_subscription = self.create_subscription(
            TwistStamped, "/jaka_driver/tool_position", self.get_robot_matrix, 5
        )
        # 订阅 transform 数据
        self.subscription = self.create_subscription(
            String, "/oculus/transforms", self.transform_callback, 10
        )

        
        # Reset 模式控制
        self.reset_mode = False
        self.prev_matrix = None
        self.prev_robot_matrix = None
        self.init_robot_matrix = None  # 初始为单位矩阵
        

        # 创建伺服控制客户端
        self.servo_client = JakaServoClientAsync()
        future = self.servo_client.enable_servo_mode()
        rclpy.spin_until_future_complete(self.servo_client, future)
        response = future.result()
        self.servo_client.get_logger().info('servo mode enable %s' % (response.message))



    def get_robot_matrix(self, msg: TwistStamped):
        """
        获取当前机器人末端的 4x4 齐次变换矩阵（由 tool_position_callback 更新）
        返回: np.ndarray (4x4)
        """
        if self.init_robot_matrix is not None:
            return self.init_robot_matrix

        self.current_tool_pose = msg
        if self.current_tool_pose is None:
            self.get_logger().warn("尚未接收到机器人末端位姿数据")
            return None

        # # 提取位置 (单位: m)
        # px = self.current_toolreset_callback_pose.twist.linear.x
        # py = self.current_tool_pose.twist.linear.y
        # pz = self.current_tool_pose.twist.linear.z

        # 提取姿态角度 (单位: 度 → 弧度)
        rx = math.radians(self.current_tool_pose.twist.angular.x)
        ry = math.radians(self.current_tool_pose.twist.angular.y)
        rz = math.radians(self.current_tool_pose.twist.angular.z)

        # 构建旋转矩阵（按 xyz 顺序）
        cx, sx = np.cos(rx), np.sin(rx)
        cy, sy = np.cos(ry), np.sin(ry)
        cz, sz = np.cos(rz), np.sin(rz)

        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

        R = Rz @ Ry @ Rx  # Rxyz

        # 构建齐次变换矩阵 T
        # T = np.eye(4)
        # T[:3, :3] = R
        # T[:3, 3] = [px, py, pz]
        self.init_robot_matrix = R
        return R


    def reset_callback(self, msg):
        # self.get_logger().info("reset_callback called")
        if self.reset_mode == msg.data:
            return
        self.get_logger().info(f"reset_callback: reset_mode = {self.reset_mode} -> {msg.data}")
        self.reset_mode = msg.data
        # self.get_logger().info(f"reset_mode: {self.reset_mode}")
        if self.reset_mode:
            self.prev_matrix = None
            self.init_robot_matrix = None
            self.prev_robot_matrix = None
            self.get_logger().info("🛑 进入 Reset 模式，暂停运动控制")
        else:
            self.get_logger().info("✅ 退出 Reset 模式")
            # 重新使能伺服模式
            future = self.servo_client.enable_servo_mode()
            rclpy.spin_until_future_complete(self.servo_client, future)
            response = future.result()
            self.servo_client.get_logger().info('servo mode enable %s' % (response.message))

    def transform_callback(self, msg):
        # self.get_logger().info(f"reset_mode: {self.reset_mode} in transform_callback")
        if self.reset_mode:
            return
        
        try:
            transforms = json.loads(msg.data)
            # print(transforms)
            matrix = np.array(transforms["r"]).reshape(4, 4)
        except Exception as e:
            self.get_logger().warn(f"Transform 解析错误: {e}")
            return
        # print(self.prev_matrix, self.prev_robot_matrix)
        if self.prev_matrix is None or self.prev_robot_matrix is None:
            self.prev_matrix = matrix
            self.prev_robot_matrix = self.init_robot_matrix
            return

        # 计算位置增量（VR -> JAKA 坐标变换）
        delta = self.compute_pose_delta(self.prev_matrix, matrix)
        
        scaled_delta = [x * self.scale for x in delta]

        future = self.servo_client.send_request(scaled_delta)
        rclpy.spin_until_future_complete(self.servo_client, future)

        # 更新历史
        self.prev_matrix = matrix

    def compute_pose_delta(self, prev, current):
        # 相对位置（m → mm）
        dp = current[:3, 3] - prev[:3, 3]
        dp_mm = dp * 1000.0

        # Oculus → JAKA 坐标轴映射
        T = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        delta_xyz = T @ dp_mm

        # 姿态增量（旋转矩阵）
        R1 = prev[:3, :3]
        R2 = current[:3, :3]
        
        # 检查矩阵是否接近奇异
        det_R1 = np.linalg.det(R1)
        if abs(det_R1) < 1e-6:  # 如果行列式太小，认为是奇异矩阵
            self.get_logger().warn(f"检测到奇异旋转矩阵，行列式: {det_R1}")
            # 返回零增量，避免错误
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        try:
            R_rel = R2 @ np.linalg.inv(R1)
        except np.linalg.LinAlgError as e:
            self.get_logger().warn(f"矩阵求逆失败: {e}")
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
        R_rel_robot = T @ R_rel @ T.T  # 转换到 JAKA 坐标系
        Robot_R1 = self.prev_robot_matrix
        Robot_R2 = self.init_robot_matrix @ R_rel_robot
        
        # 检查机器人矩阵是否奇异
        det_Robot_R1 = np.linalg.det(Robot_R1)
        if abs(det_Robot_R1) < 1e-6:
            self.get_logger().warn(f"检测到奇异机器人矩阵，行列式: {det_Robot_R1}")
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
        try:
            R_rel_angle = np.linalg.inv(Robot_R1) @ Robot_R2
        except np.linalg.LinAlgError as e:
            self.get_logger().warn(f"机器人矩阵求逆失败: {e}")
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
        d_roll, d_pitch, d_yaw = self.rotation_matrix_to_rpy(R_rel_angle)
        
        return [
            float(delta_xyz[0]),
            float(delta_xyz[1]),
            float(delta_xyz[2]),
            self.normalize_angle(d_roll),
            self.normalize_angle(d_pitch),
            self.normalize_angle(d_yaw),
        ]

    def rotation_matrix_to_rpy(self, R):
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
        yaw = math.atan2(R[1, 0], R[0, 0])
        return roll, pitch, yaw

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi


def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()