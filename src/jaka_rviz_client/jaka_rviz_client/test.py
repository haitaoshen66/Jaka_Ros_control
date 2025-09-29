from jaka_msgs.srv import GetIK

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(GetIK, 'jaka_rviz_driver/get_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetIK.Request()

    def send_request(self,cartesian_pose):
        self.req.cartesian_pose = cartesian_pose
        return self.cli.call_async(self.req)


def main(args=None):
    try:
        rclpy.init(args=args)
        minimal_client = MinimalClientAsync()
        future = minimal_client.send_request([0.0, 0.0, 0.0, -0.28, -0.2, 0.4])
        rclpy.spin_until_future_complete(minimal_client, future)
        response = future.result()
        # response.message:atring{Failed , Success}
        minimal_client.get_logger().info(response.message)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()