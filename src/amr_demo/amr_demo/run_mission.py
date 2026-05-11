#!/usr/bin/env python3

import argparse
import math
import sys
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import os


MISSION_PICKUPS = {
    'grocery': 'SUPERMARKET',
    'food': 'RESTAURANT',
    'fire': 'FIRE_STATION',
    'medical': 'PHARMACY',
}


def yaw_to_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def load_landmarks(self):
        package_share = get_package_share_directory('amr_demo')
        path = os.path.join(package_share, 'config', 'landmarks.yaml')

        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        return data['landmarks']

    def send_goal(self, name, pose):
        if not self.client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('/navigate_to_pose action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(pose['x'])
        goal_msg.pose.pose.position.y = float(pose['y'])
        goal_msg.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(float(pose['yaw']))
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'Going to {name}: x={pose["x"]:.2f}, y={pose["y"]:.2f}, yaw={pose["yaw"]:.2f}'
        )

        send_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected: {name}')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result is None or result.status != 4:
            self.get_logger().error(f'Failed to reach {name}')
            return False

        self.get_logger().info(f'Reached {name}')
        return True

    def run_mission(self, mission_type, home):
        mission_type = mission_type.lower()
        home = home.upper()

        if mission_type not in MISSION_PICKUPS:
            self.get_logger().error(f'Invalid mission type: {mission_type}')
            return False

        if not home.startswith('HOUSE_'):
            self.get_logger().error('Home must be HOUSE_1, HOUSE_2, HOUSE_3, HOUSE_4, or HOUSE_5')
            return False

        landmarks = self.load_landmarks()

        pickup = MISSION_PICKUPS[mission_type]
        sequence = ['DOCK', pickup, home, 'DOCK']

        self.get_logger().info(f'Mission sequence: {" -> ".join(sequence)}')

        for waypoint in sequence:
            if waypoint not in landmarks:
                self.get_logger().error(f'Missing landmark in database: {waypoint}')
                return False

            if not self.send_goal(waypoint, landmarks[waypoint]):
                self.get_logger().error('Mission failed')
                return False

        self.get_logger().info('Mission completed successfully')
        return True


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--type', required=True, choices=['grocery', 'food', 'fire', 'medical'])
    parser.add_argument('--home', required=True, choices=['HOUSE_1', 'HOUSE_2', 'HOUSE_3', 'HOUSE_4', 'HOUSE_5'])
    parsed = parser.parse_args()

    rclpy.init(args=args)
    node = MissionPlanner()

    try:
        success = node.run_mission(parsed.type, parsed.home)
    except KeyboardInterrupt:
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
