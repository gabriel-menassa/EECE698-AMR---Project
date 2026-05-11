#!/usr/bin/env python3

import math
import random
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


ROBOT_1_LOOP = [
    (-6.0,  4.5),
    (-2.5,  4.5),
    ( 2.5,  4.5),
    ( 6.0,  4.5),
    ( 6.0,  2.0),
    ( 2.5,  2.0),
    (-2.5,  2.0),
    (-6.0,  2.0),
]

ROBOT_2_LOOP = [
    (-6.0, -2.0),
    (-2.5, -2.0),
    ( 2.5, -2.0),
    ( 6.0, -2.0),
    ( 6.0, -5.5),
    ( 2.5, -5.5),
    (-2.5, -5.5),
    (-6.0, -5.5),
]


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class LoopTrafficRobot:
    def __init__(self, node, name, cmd_topic, odom_topic, loop_points):
        self.node = node
        self.name = name
        self.loop_points = loop_points

        self.pub = node.create_publisher(Twist, cmd_topic, 10)
        self.sub = node.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.x = None
        self.y = None
        self.yaw = None

        self.index = 0
        self.direction = random.choice([1, -1])

        self.last_x = None
        self.last_y = None
        self.last_progress_time = time.time()

        self.recovery_until = 0.0
        self.recovery_turn = 0.0

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def nearest_loop_index(self):
        dists = [
            math.hypot(px - self.x, py - self.y)
            for px, py in self.loop_points
        ]
        return min(range(len(dists)), key=lambda i: dists[i])

    def initialize_index_once(self):
        if self.index == 0 and self.x is not None:
            self.index = self.nearest_loop_index()

    def next_index(self):
        return (self.index + self.direction) % len(self.loop_points)

    def occasionally_change_direction(self):
        if random.random() < 0.08:
            self.direction *= -1
            self.node.get_logger().info(f'{self.name} changed loop direction.')

    def check_stuck(self):
        now = time.time()

        if self.last_x is None:
            self.last_x = self.x
            self.last_y = self.y
            self.last_progress_time = now
            return

        moved = math.hypot(self.x - self.last_x, self.y - self.last_y)

        if moved > 0.20:
            self.last_x = self.x
            self.last_y = self.y
            self.last_progress_time = now
            return

        if now - self.last_progress_time > 3.0:
            self.recovery_until = now + 1.1
            self.recovery_turn = random.choice([-1.0, 1.0]) * 2.2
            self.direction *= -1

            self.node.get_logger().info(
                f'{self.name} recovery: reverse/turn and flip direction.'
            )

            self.last_x = self.x
            self.last_y = self.y
            self.last_progress_time = now

    def update(self):
        if self.x is None or self.y is None or self.yaw is None:
            self.publish_cmd(0.0, 0.0)
            return

        self.initialize_index_once()

        now = time.time()

        if now < self.recovery_until:
            self.publish_cmd(-0.75, self.recovery_turn)
            return

        self.check_stuck()

        tx, ty = self.loop_points[self.index]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < 0.70:
            self.index = self.next_index()
            self.occasionally_change_direction()
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(desired_yaw - self.yaw)

        if abs(yaw_error) > 0.65:
            linear = 0.03
            angular = max(min(2.4 * yaw_error, 2.0), -2.0)
        else:
            linear = 0.85
            angular = max(min(1.7 * yaw_error, 1.4), -1.4)

        self.publish_cmd(linear, angular)


class TrafficController(Node):
    def __init__(self):
        super().__init__('traffic_controller')

        self.robot1 = LoopTrafficRobot(
            self,
            'robot_traffic_1',
            '/robot_traffic_1/cmd_vel',
            '/model/robot_traffic_1/odometry',
            ROBOT_1_LOOP
        )

        self.robot2 = LoopTrafficRobot(
            self,
            'robot_traffic_2',
            '/robot_traffic_2/cmd_vel',
            '/model/robot_traffic_2/odometry',
            ROBOT_2_LOOP
        )

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info('Safe loop traffic controller started.')

    def loop(self):
        self.robot1.update()
        self.robot2.update()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()