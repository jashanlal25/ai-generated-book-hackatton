#!/usr/bin/env python3
"""
Humanoid Status Subscriber
Receives and logs status messages demonstrating the ROS 2 subscriber pattern.

Run with: python3 subscriber.py
Requires: ROS 2 Humble, rclpy, std_msgs
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HumanoidStatusSubscriber(Node):
    def __init__(self):
        super().__init__('humanoid_status_subscriber')
        self.subscription = self.create_subscription(
            String,
            'humanoid_status',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidStatusSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
