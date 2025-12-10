#!/usr/bin/env python3
"""
Joint Calculator Service Server
Demonstrates ROS 2 service pattern with a simple calculation service.

Run with: python3 service_server.py
Requires: ROS 2 Humble, rclpy, example_interfaces
"""
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class JointCalculatorService(Node):
    def __init__(self):
        super().__init__('joint_calculator_service')
        self.srv = self.create_service(
            AddTwoInts,
            'calculate_joint_torque',
            self.calculate_callback)
        self.get_logger().info('Joint calculator service ready')

    def calculate_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = JointCalculatorService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
