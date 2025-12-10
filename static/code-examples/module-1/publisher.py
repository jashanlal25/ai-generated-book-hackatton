#!/usr/bin/env python3
"""
Humanoid Status Publisher
Publishes periodic status messages demonstrating the ROS 2 publisher pattern.

Run with: python3 publisher.py
Requires: ROS 2 Humble, rclpy, std_msgs
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HumanoidStatusPublisher(Node):
    def __init__(self):
        super().__init__('humanoid_status_publisher')
        self.publisher_ = self.create_publisher(String, 'humanoid_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Humanoid status update #{self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
