import rclpy
from rclpy.node import Node
from jaka_msgs.srv import GetIK, Move
import csv
import time

# 替換為您的 CSV 文件路徑
CSV_FILE_PATH = '/home/dija/DIJA/questVR_ws/src/oculus_reader/scripts/right_hand_xyz_delta.csv'

class RobotAutomationNode(Node):

    def __init__(self, initial_joint_position):
        super().__init__('robot_automation_node')
        
        # 保存初始关节位置
        self.current_joint_position = initial_joint_position
        self.get_logger().info(f'Using initial joint position: {self.current_joint_position}')

        # 创建 GetIK 服务客户端
        self.get_logger().info('Creating GetIK service client...')
        self.get_ik_client = self.create_client(GetIK, '/jaka_driver/get_ik')
        while not self.get_ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GetIK service not available, waiting again...')
        self.get_logger().info('GetIK service client created.')

        # 创建 JointMove 服务客户端
        self.get_logger().info('Creating JointMove service client...')
        self.joint_move_client = self.create_client(Move, '/jaka_driver/joint_move')
        while not self.joint_move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('JointMove service not available, waiting again...')
        self.get_logger().info('JointMove service client created.')

        self.get_logger().info('Robot Automation Node initialized.')

    def process_csv_data(self):
        init_position = [154,-396.7,211,-0.785,-1.57,-2.36]
        with open(CSV_FILE_PATH, 'r') as csvfile:
            csv_reader = csv.reader(csvfile)
            header = next(csv_reader)  # 读取 CSV 头部 (如果有的话)
            self.get_logger().info(f'CSV Header: {header}')

            for i, row in enumerate(csv_reader):
                self.get_logger().info(f'Processing row {i+1}: {row}')

                try:
                    # 解析 CSV 数据
                    # 假设 CSV 顺序是 x, y, z, roll, pitch, yaw
                    init_position[0] = float(row[4]) + init_position[0]  # 假设 CSV 中的 x, y, z 是相对于初始位置的增量
                    init_position[1] = float(row[5]) + init_position[1]
                    init_position[2] = float(row[6]) + init_position[2]
                    init_position[3] = float(row[1]) + init_position[3]
                    init_position[4] = float(row[2]) + init_position[4]
                    init_position[5] = float(row[3]) + init_position[5]
                    x = init_position[0]
                    y = init_position[1]
                    z = init_position[2]
                    roll = init_position[3]
                    pitch = init_position[4]
                    yaw = init_position[5]

                    self.get_logger().info(f'Parsed values - x: {x}, y: {y}, z: {z}, roll: {roll}, pitch: {pitch}, yaw: {yaw}')
                    # 构建 cartesian_pose
                    cartesian_pose = [x, y, z, roll, pitch, yaw]
                    self.get_logger().info(f'Cartesian Pose (scaled x,y,z): {cartesian_pose}')

                    # 调用 GetIK 服务
                    self.get_logger().info(f'Calling GetIK service with ref_joint: {self.current_joint_position} and cartesian_pose: {cartesian_pose}')
                    get_ik_request = GetIK.Request()
                    get_ik_request.ref_joint = list(self.current_joint_position)
                    get_ik_request.cartesian_pose = cartesian_pose
                    # 同步调用服务
                    get_ik_future = self.get_ik_client.call_async(get_ik_request)
                    rclpy.spin_until_future_complete(self, get_ik_future)
                    get_ik_response = get_ik_future.result()

                    if get_ik_response.joint[0] == 9999.0: # 检查 IK 错误标志
                        self.get_logger().error(f'GetIK service failed for row {i+1}: {get_ik_response.message}. Skipping to next row.')
                        continue
                    
                    new_joint_pose = list(get_ik_response.joint)
                    self.get_logger().info(f'Successfully got IK solution: {new_joint_pose}')
                    
                    # 更新当前关节位置，作为下一次IK的参考
                    self.current_joint_position = new_joint_pose

                    # 调用 JointMove 服务
                    self.get_logger().info(f'Calling JointMove service with pose: {new_joint_pose}')
                    move_request = Move.Request()
                    move_request.pose = new_joint_pose
                    move_request.has_ref = False
                    move_request.ref_joint = [0.0]
                    move_request.mvvelo = 1.0 # 运动速度
                    move_request.mvacc = 1.0  # 运动加速度
                    move_request.mvtime = 0.0
                    move_request.mvradii = 0.0
                    move_request.coord_mode = 0
                    move_request.index = 0

                    # 同步调用服务
                    move_future = self.joint_move_client.call_async(move_request)
                    rclpy.spin_until_future_complete(self, move_future)
                    move_response = move_future.result()
                    self.get_logger().info(f'JointMove service response: {move_response.message}')

                    # 等待机器人移动完成
                    # time.sleep(3) # 简单延迟，根据机器人运动时间调整

                except Exception as e:
                    self.get_logger().error(f'Error processing row {i+1}: {e}')
                    continue # 跳过当前错误行，继续处理下一行
        
        self.get_logger().info('Finished processing all CSV data.')


def main(args=None):
    rclpy.init(args=args)
    
    # 在这里设置初始关节角度
    initial_joint_position = [0.0, 0.0, 1.57, -1.57, 1.57, 0.0]  # 请替换为实际的初始关节角度
    
    node = RobotAutomationNode(initial_joint_position)
    try:
        # 直接调用处理方法，顺序执行
        node.process_csv_data()
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()