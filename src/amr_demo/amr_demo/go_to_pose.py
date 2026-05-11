#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class GoToPoseClient(Node):
    def __init__(self):
        super().__init__('go_to_pose_client')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self, x, y, yaw):
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        if not self.client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('NavigateToPose action server not available.')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(float(yaw))
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'Sending goal: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad'
        )

        send_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )

        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error('Goal request failed.')
            return False

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return False

        self.get_logger().info('Goal accepted. Navigating...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result is None:
            self.get_logger().error('No result returned.')
            return False

        status = result.status

        if status == 4:
            self.get_logger().info('Goal succeeded.')
            return True

        self.get_logger().error(f'Goal failed with status code: {status}')
        return False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        nav_time = feedback.navigation_time.sec
        self.get_logger().info(
            f'Distance remaining: {distance:.2f} m | Navigation time: {nav_time}s'
        )


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, default=0.0)
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    node = GoToPoseClient()

    try:
        success = node.send_goal(parsed_args.x, parsed_args.y, parsed_args.yaw)
    except KeyboardInterrupt:
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
