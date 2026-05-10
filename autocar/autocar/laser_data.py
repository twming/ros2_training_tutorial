#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

class LaserDataNode(Node):
    def __init__(self):
        super().__init__('laser_data_node')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos_profile,
            #10
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
