#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.first_pose_received = False

    def odom_callback(self, msg):
        if not self.first_pose_received:
            self.get_logger().info('Setting initial pose based on first received odometry data.')
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.header.frame_id = "map"  # Or whatever frame you are using
            initial_pose.pose.pose = msg.pose.pose
            initial_pose.pose.pose.position.x=msg.pose.pose.position.x+2.0
            initial_pose.pose.pose.position.y=msg.pose.pose.position.y+0.5
            self.publisher.publish(initial_pose)
            self.first_pose_received = True
            self.get_logger().info('Initial pose has been set and published.')

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()