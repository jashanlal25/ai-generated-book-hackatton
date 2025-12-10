#!/usr/bin/env python3
"""
Joint Calculator Service Client
Demonstrates making service calls in ROS 2.

Run with: python3 service_client.py
Requires: ROS 2 Humble, rclpy, example_interfaces
"""
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class JointCalculatorClient(Node):
    def __init__(self):
        super().__init__('joint_calculator_client')
        self.cli = self.create_client(AddTwoInts, 'calculate_joint_torque')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    client = JointCalculatorClient()
    future = client.send_request(10, 5)
    rclpy.spin_until_future_complete(client, future)
    result = future.result()
    client.get_logger().info(f'Result: {result.sum}')
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
