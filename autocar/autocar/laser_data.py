#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserDataNode(Node):
    def __init__(self):
        super().__init__('laser_data_node')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def laser_callback(self, msg):
        print('s1 [0]:', msg.ranges[0])
        print('s2 [90]:', msg.ranges[90])
        print('s3 [180]:', msg.ranges[180])
        print('s4 [270]:', msg.ranges[270])
        print('s5 [359]:', msg.ranges[359])

def main(args=None):
    rclpy.init(args=args)
    laser_data_node = LaserDataNode()
    rclpy.spin(laser_data_node)
    laser_data_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()