#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.move = Twist()

    def scan_callback(self, msg):
        print('s1 [0]:', msg.ranges[0])
        
        if msg.ranges[0] > 0.5:
            self.move.linear.x = 0.3
            self.move.angular.z = 0.0
        else:
            self.move.linear.x = 0.0
            self.move.angular.z = 0.0
        
        self.publisher.publish(self.move)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()
    rclpy.spin(obstacle_avoidance_node)
    obstacle_avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()