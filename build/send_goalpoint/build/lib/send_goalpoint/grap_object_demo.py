import sys
import rclpy
from rclpy.node import Node
from roarm_moveit.srv import MovePointCmd, setgrippercmd


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


class SetGripperClient(Node):
    def __init__(self):
        super().__init__('set_gripper_client_node')
        self.client = self.create_client(setgrippercmd, 'set_gripper_cmd')
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('SetGripper service not available, waiting...')
        self.get_logger().info('SetGripper service available.')

    def send_request(self, width):
        request = setgrippercmd.Request()
        request.width = width
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)

    paint_points = [(0.2, 0.0, 0.0), (0.2, 0.0, 0.3), (0.2, 0.1, 0.3)]

    move_client = MovePointClient()
    gripper_client = SetGripperClient()

    # Открываем гриппер перед движением
    gripper_client.send_request(0.5)
    print("Gripper opened to 0.5")

    for point in paint_points:
        x, y, z = point
        move_client.send_request(x, y, z)
        print(f"Moved to point: {point}")

    # После последней точки — сжимаем гриппер
    gripper_client.send_request(0.2)
    print("Gripper closed to 0.2")

    move_client.destroy_node()
    gripper_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
