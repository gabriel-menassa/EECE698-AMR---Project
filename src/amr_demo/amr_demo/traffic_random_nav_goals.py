#!/usr/bin/env python3

import argparse
import math
import random
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float


SAFE_WAYPOINTS = [
    Waypoint('dock', -0.81, -5.46, -1.57),
    Waypoint('pharmacy', -4.50, -4.69, 3.14),
    Waypoint('fire_station', 4.50, -2.54, 0.0),
    Waypoint('supermarket', -6.85, 5.65, 3.14),
    Waypoint('restaurant', 3.50, 7.99, 0.37),
    Waypoint('house_1', -2.96, -0.05, 3.14),
    Waypoint('house_2', -5.51, 5.49, 1.82),
    Waypoint('house_3', 2.44, 6.60, 1.71),
    Waypoint('house_4', 1.11, 0.84, 0.12),
    Waypoint('house_5', -0.53, 1.21, 1.78),
    Waypoint('west_mid', -6.00, 2.00, 0.0),
    Waypoint('west_upper', -6.00, 4.50, 0.0),
    Waypoint('mid_upper_west', -2.50, 4.50, 0.0),
    Waypoint('mid_upper_east', 2.50, 4.50, 3.14),
    Waypoint('east_upper', 6.00, 4.50, 3.14),
    Waypoint('east_mid', 6.00, 2.00, 3.14),
    Waypoint('mid_lower_west', -2.50, -2.00, 0.0),
    Waypoint('mid_lower_east', 2.50, -2.00, 3.14),
    Waypoint('west_lower', -6.00, -5.50, 0.0),
    Waypoint('east_lower', 6.00, -5.50, 3.14),
]


INITIAL_POSES = {
    'robot_traffic_1': (-9.0, 2.5, 0.0),
    'robot_traffic_2': (9.0, 2.5, 3.14),
}


def yaw_to_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def status_name(status):
    names = {
        STATUS_SUCCEEDED: 'SUCCEEDED',
        STATUS_CANCELED: 'CANCELED',
        STATUS_ABORTED: 'ABORTED',
    }
    return names.get(status, f'STATUS_{status}')


class TrafficRobotNavigator:
    def __init__(self, node, robot_name, waypoints, goal_delay, goal_timeout):
        self.node = node
        self.robot_name = robot_name
        self.waypoints = waypoints
        self.goal_delay = goal_delay
        self.goal_timeout = goal_timeout

        self.client = ActionClient(
            node,
            NavigateToPose,
            f'/{robot_name}/navigate_to_pose',
        )

        self.active = False
        self.goal_handle = None
        self.goal_id = 0
        self.active_goal_id = None
        self.sent_at = None
        self.next_goal_time = time.monotonic()
        self.current_waypoint = None
        self.last_waypoint = None
        self.wait_started = time.monotonic()
        self.last_wait_log = 0.0

    def tick(self, avoid_waypoint=None):
        now = time.monotonic()

        if self.active:
            self.cancel_timed_out_goal(now)
            return

        if now < self.next_goal_time:
            return

        if not self.client.wait_for_server(timeout_sec=0.1):
            self.log_waiting_for_server(now)
            return

        self.send_next_goal(avoid_waypoint)

    def cancel_timed_out_goal(self, now):
        if self.goal_handle is None or self.sent_at is None:
            return

        if now - self.sent_at <= self.goal_timeout:
            return

        self.node.get_logger().warn(
            f'{self.robot_name}: canceling goal {self.current_waypoint.name} '
            f'after {self.goal_timeout:.0f}s timeout.'
        )
        self.goal_handle.cancel_goal_async()
        self.finish_goal(goal_id=self.active_goal_id)

    def log_waiting_for_server(self, now):
        if now - self.last_wait_log < 5.0:
            return

        waited = now - self.wait_started
        self.node.get_logger().info(
            f'{self.robot_name}: waiting for /{self.robot_name}/navigate_to_pose '
            f'({waited:.0f}s).'
        )
        self.last_wait_log = now

    def choose_waypoint(self, avoid_waypoint):
        candidates = [
            waypoint for waypoint in self.waypoints
            if waypoint != self.last_waypoint and waypoint != avoid_waypoint
        ]
        if not candidates:
            candidates = [waypoint for waypoint in self.waypoints if waypoint != self.last_waypoint]
        if not candidates:
            candidates = list(self.waypoints)
        return random.choice(candidates)

    def send_next_goal(self, avoid_waypoint):
        waypoint = self.choose_waypoint(avoid_waypoint)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint.x
        goal_msg.pose.pose.position.y = waypoint.y
        goal_msg.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(waypoint.yaw)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.node.get_logger().info(
            f'{self.robot_name}: sending {waypoint.name} '
            f'(x={waypoint.x:.2f}, y={waypoint.y:.2f}, yaw={waypoint.yaw:.2f}).'
        )

        self.active = True
        self.current_waypoint = waypoint
        self.sent_at = time.monotonic()
        self.goal_id += 1
        self.active_goal_id = self.goal_id

        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda done, goal_id=self.active_goal_id: self.goal_response_callback(done, goal_id)
        )

    def goal_response_callback(self, future, goal_id):
        if goal_id != self.active_goal_id:
            return

        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            name = self.current_waypoint.name if self.current_waypoint else 'unknown'
            self.node.get_logger().warn(f'{self.robot_name}: goal rejected: {name}.')
            self.finish_goal(goal_id=goal_id, retry_delay=3.0)
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done, result_goal_id=goal_id: self.result_callback(done, result_goal_id)
        )

    def result_callback(self, future, goal_id):
        if goal_id != self.active_goal_id:
            return

        result = future.result()
        waypoint = self.current_waypoint
        waypoint_name = waypoint.name if waypoint else 'unknown'

        if result is None:
            self.node.get_logger().warn(f'{self.robot_name}: no result for {waypoint_name}.')
            self.finish_goal(goal_id=goal_id, retry_delay=3.0)
            return

        if result.status == STATUS_SUCCEEDED:
            self.node.get_logger().info(f'{self.robot_name}: reached {waypoint_name}.')
        else:
            self.node.get_logger().warn(
                f'{self.robot_name}: {waypoint_name} finished with {status_name(result.status)}.'
            )

        self.finish_goal(goal_id=goal_id)

    def finish_goal(self, goal_id=None, retry_delay=None):
        if goal_id is not None and goal_id != self.active_goal_id:
            return

        if self.current_waypoint is not None:
            self.last_waypoint = self.current_waypoint

        delay = self.goal_delay if retry_delay is None else retry_delay
        self.active = False
        self.goal_handle = None
        self.active_goal_id = None
        self.sent_at = None
        self.current_waypoint = None
        self.next_goal_time = time.monotonic() + delay


class TrafficRandomNavGoals(Node):
    def __init__(
        self,
        goal_delay,
        goal_timeout,
        initial_pose_repeats,
        initial_pose_period,
        start_delay,
        publish_initial_poses,
    ):
        super().__init__('traffic_random_nav_goals')

        self.robots = [
            TrafficRobotNavigator(
                self,
                'robot_traffic_1',
                SAFE_WAYPOINTS,
                goal_delay,
                goal_timeout,
            ),
            TrafficRobotNavigator(
                self,
                'robot_traffic_2',
                SAFE_WAYPOINTS,
                goal_delay,
                goal_timeout,
            ),
        ]

        self.initial_pose_publishers = {
            robot_name: self.create_publisher(
                PoseWithCovarianceStamped,
                f'/{robot_name}/initialpose',
                10,
            )
            for robot_name in INITIAL_POSES
        }
        self.initial_pose_repeats = initial_pose_repeats
        self.initial_pose_period = initial_pose_period
        self.start_delay = start_delay
        self.publish_initial_poses = publish_initial_poses
        self.initial_pose_count = 0
        self.last_initial_pose_time = 0.0
        self.goals_enabled_at = None

        self.timer = self.create_timer(0.5, self.tick)
        self.get_logger().info(
            f'Traffic random Nav2 goals started with {len(SAFE_WAYPOINTS)} waypoints.'
        )

    def tick(self):
        if not self.initialization_complete():
            return

        for robot in self.robots:
            avoid = next(
                (
                    other.current_waypoint for other in self.robots
                    if other is not robot and other.current_waypoint is not None
                ),
                None,
            )
            robot.tick(avoid_waypoint=avoid)

    def initialization_complete(self):
        now = time.monotonic()

        if self.publish_initial_poses and self.initial_pose_count < self.initial_pose_repeats:
            if now - self.last_initial_pose_time >= self.initial_pose_period:
                self.publish_initial_pose_set()
                self.initial_pose_count += 1
                self.last_initial_pose_time = now
            return False

        if self.goals_enabled_at is None:
            self.goals_enabled_at = now + self.start_delay
            self.get_logger().info(
                f'Traffic initial poses done. Sending goals in {self.start_delay:.1f}s.'
            )
            return False

        return now >= self.goals_enabled_at

    def publish_initial_pose_set(self):
        for robot_name, pose in INITIAL_POSES.items():
            x, y, yaw = pose
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = 0.0

            qz, qw = yaw_to_quaternion(yaw)
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw

            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.07

            self.initial_pose_publishers[robot_name].publish(msg)

        self.get_logger().info(
            f'Published traffic initial poses '
            f'({self.initial_pose_count + 1}/{self.initial_pose_repeats}).'
        )


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal-delay', type=float, default=2.0)
    parser.add_argument('--goal-timeout', type=float, default=120.0)
    parser.add_argument('--initial-pose-repeats', type=int, default=6)
    parser.add_argument('--initial-pose-period', type=float, default=0.5)
    parser.add_argument('--start-delay', type=float, default=3.0)
    parser.add_argument('--no-initial-poses', action='store_true')
    parser.add_argument('--seed', type=int, default=None)
    parsed, _ = parser.parse_known_args(args=args)

    if parsed.seed is not None:
        random.seed(parsed.seed)

    rclpy.init(args=args)
    node = TrafficRandomNavGoals(
        goal_delay=parsed.goal_delay,
        goal_timeout=parsed.goal_timeout,
        initial_pose_repeats=parsed.initial_pose_repeats,
        initial_pose_period=parsed.initial_pose_period,
        start_delay=parsed.start_delay,
        publish_initial_poses=not parsed.no_initial_poses,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
