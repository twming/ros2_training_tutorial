#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('vel_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_velocity)
        self.move = Twist()

    def publish_velocity(self):
        self.move.linear.x = 0.1
        self.move.angular.z = 0.2
        self.publisher.publish(self.move)
        self.get_logger().info(f'Publishing: linear x={self.move.linear.x} angular z={self.move.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()