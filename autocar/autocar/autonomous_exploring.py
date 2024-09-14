#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ExploringNode(Node):
    def __init__(self):
        super().__init__('exploring_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.front_dist = 0.0
        self.left_dist = 0.0
        self.right_dist = 0.0

    def scan_callback(self, msg):
        # Update distance measurements
        self.front_dist = msg.ranges[0]
        self.left_dist = msg.ranges[45]
        self.right_dist = msg.ranges[315]
        self.drive_logic()

    def drive_logic(self):
        move = Twist()
        if self.front_dist >= 0.5 and self.left_dist >= 0.5 and self.right_dist >= 0.5:
            move.linear.x = 0.3
            move.angular.z = 0.0
            self.get_logger().info('Going Straight')
        elif self.left_dist < 0.5:
            move.linear.x = 0.0
            move.angular.z = -1.0  # 60 degrees to right
            self.get_logger().info('Turning 60 Right')
        elif self.right_dist < 0.5:
            move.linear.x = 0.0
            move.angular.z = 1.0  # 60 degrees to left
            self.get_logger().info('Turning 60 Left')
        else:
            move.linear.x = 0.0
            move.angular.z = 1.57  # 90 degrees to left
            self.get_logger().info('Turning 90 Left')

        self.publisher_.publish(move)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ExploringNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()