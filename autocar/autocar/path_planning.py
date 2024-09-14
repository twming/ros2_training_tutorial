#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status = GoalStatus.STATUS_EXECUTING
        self.waypoints = [
            [(2.0, 2.5, 0.0), (0.0, 0.0, 0.0, 1.0)],
            [(4.0, 0.5, 0.0), (0.0, 0.0, 0.0, 1.0)],
            [(2.0, -1.5, 0.0), (0.0, 0.0, 0.0, 1.0)],
            [(0.0, 0.5, 0.0), (0.0, 0.0, 0.0, 1.0)],
        ]

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = pose[0][0]
        goal_msg.pose.pose.position.y = pose[0][1]
        goal_msg.pose.pose.position.z = pose[0][2]
        goal_msg.pose.pose.orientation.x = pose[1][0]
        goal_msg.pose.pose.orientation.y = pose[1][1]
        goal_msg.pose.pose.orientation.z = pose[1][2]
        goal_msg.pose.pose.orientation.w = pose[1][3]
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f'Going for goal: {goal_msg.pose.pose}')
        
        self.client.wait_for_server()
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.status=GoalStatus.STATUS_SUCCEEDED

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    
    # Loop to send goals
    while rclpy.ok():
        for pose in navigation_node.waypoints:
            navigation_node.status=GoalStatus.STATUS_EXECUTING
            navigation_node.send_goal(pose)
            while navigation_node.status != GoalStatus.STATUS_SUCCEEDED:
                rclpy.spin_once(navigation_node)
                
            navigation_node.get_logger().info(f' -------------------------------Start Next Goal -------------------------------')

    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()