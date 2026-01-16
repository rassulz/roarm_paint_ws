import rclpy
from rclpy.node import Node
from roarm_moveit.srv import MovePointCmd
from std_msgs.msg import Float32
import time


class MovePointClient(Node):
    def __init__(self):
        super().__init__('move_point_client_node')
        self.client = self.create_client(MovePointCmd, 'move_point_cmd')

        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('MovePointCmd service not available, waiting...')
        self.get_logger().info('MovePointCmd service available.')

    def send_request(self, x, y, z):
        request = MovePointCmd.Request()
        request.x = x
        request.y = y
        request.z = z
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


class GripperPublisher(Node):
    def __init__(self):
        super().__init__('gripper_publisher_node')
        self.publisher = self.create_publisher(Float32, '/gripper_cmd', 10)

    def set_gripper(self, value):
        msg = Float32()
        msg.data = value
        self.publisher.publish(msg)
        self.get_logger().info(f'Gripper set to {value}')


def main(args=None):
    rclpy.init(args=args)

    paint_points = [(0.2, 0.0, 0.0), (0.2, 0.1, 0.0)]

    move_client = MovePointClient()
    gripper = GripperPublisher()

    # Открыть гриппер перед началом движения
    gripper.set_gripper(0.5)
    time.sleep(5.0)

    for point in paint_points:
        x, y, z = point
        move_client.send_request(x, y, z)
        print(f"Moved to point: {point}")

        time.sleep(5.0)
        gripper.set_gripper(0.2)
        print("Gripper closed to 0.2")
        
        time.sleep(1.0)

    # Сжать гриппер после последней точки
    gripper.set_gripper(0.5)
    print("Gripper closed to 0.5")

    move_client.destroy_node()
    gripper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
