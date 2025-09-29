from jaka_msgs.srv import ServoMoveEnable,ServoMove
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import csv

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


def main(args=None):
    try:
        rclpy.init(args=args)
        minimal_client = JakaServoClientAsync()
        future = minimal_client.enable_servo_mode()
        rclpy.spin_until_future_complete(minimal_client, future)
        response = future.result()
        minimal_client.get_logger().info('servo mode enable %s' % (response.message))
        pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
        N = 100
        for _ in range(N):
            future = minimal_client.send_request(pose)
            rclpy.spin_until_future_complete(minimal_client, future)
            response = future.result()
            minimal_client.get_logger().info('request result %s' % (response.message))
        # csv_file_path = "/home/dija/DIJA/questVR_ws/src/oculus_reader/scripts/right_hand_xyz.csv"
        # minimal_client.get_logger().info(f'Reading poses from {csv_file_path}')
        
        # # 读取CSV中的每一行并发送请求
        # with open(csv_file_path, 'r') as csvfile:
        #     csv_reader = csv.reader(csvfile)
        #     next(csv_reader) 
        #     for row in csv_reader:
        #         # 将字符串转换为浮点数
        #         pose = [float(val) for val in row[1:7]]  # 跳过第一列，只取1-6列的数据
                
        #         # 检查姿态是否有6个元素
        #         if len(pose) != 6:
        #             minimal_client.get_logger().warn(f'Invalid pose length: {len(pose)}. Expected 6 values. Skipping.')
        #             continue
        #         pose[3]*=0.001
        #         pose[4]*=0.001
        #         pose[5]*=0.001
        #         # 发送姿态请求
        #         future = minimal_client.send_request(pose)
        #         rclpy.spin_until_future_complete(minimal_client, future)
        #         response = future.result()
        #         minimal_client.get_logger().info(f'request result {response.message} for pose {pose}')
            
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()