import sys
import rclpy
from rclpy.node import Node
from roarm_moveit.srv import MovePointCmd
import time

class MovePointClient(Node):
    def __init__(self):
        super().__init__('move_point_client_node')
        self.client = self.create_client(MovePointCmd, 'move_point_cmd')

        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Service Available')

    def send_request(self, x, y, z):
        request = MovePointCmd.Request()
        request.x = x
        request.y = y
        request.z = z

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)

    # if len(sys.argv) != 4:
        # print("Usage: enter x y z")
        # return

    # x = float(sys.argv[1])
    # y = float(sys.argv[2])
    # z = float(sys.argv[3])

    paint_point = [(0.2, 0.0, 0.0), (0.2, 0.0, 0.3), (0.2, 0.1, 0.3), (0.25, 0.25, 0.25), (0.2, 0.0, 0.25)]

    client_node = MovePointClient()

    time.sleep(5.0)
    for point in paint_point:
        x, y, z = point
        response = client_node.send_request(x, y, z)
        print(f"Moved to point: {point}")
        time.sleep(0.5)
    time.sleep(5.0)



    
    

    if response.success:
        client_node.get_logger().info(f'Success: {response.message}')
    else:
        client_node.get_logger().warn(f'Failed: {response.message}')

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()