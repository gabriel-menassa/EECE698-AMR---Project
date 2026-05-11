#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class OdomSweepExplorer(Node):
    def __init__(self):
        super().__init__("odom_sweep_explorer")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, "/lidar", self.scan_cb, 10)

        self.x = None
        self.y = None
        self.yaw = None
        self.front = float("inf")

        # Safe sweep route for your current world.sdf.
        # Avoids buildings and walls while passing near all major map regions.
        self.waypoints = [
            (0.0, -7.0),
            (-5.0, -7.0),
            (-9.0, -7.0),
            (-9.0, -3.5),
            (-9.0, 1.0),
            (-9.0, 6.0),
            (-5.0, 6.0),
            (-1.5, 2.0),
            (1.5, 2.0),
            (5.0, 6.0),
            (9.0, 6.0),
            (9.0, 1.0),
            (9.0, -3.5),
            (9.0, -7.0),
            (5.0, -7.0),
            (0.0, -7.0),
        ]

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

    def scan_cb(self, msg):
        ranges = list(msg.ranges)
        n = len(ranges)
        vals = ranges[int(0.43 * n):int(0.57 * n)]
        vals = [v for v in vals if math.isfinite(v) and 0.05 < v < 20.0]
        self.front = min(vals) if vals else float("inf")

    def cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)

    def stop(self):
        self.cmd(0.0, 0.0)
        time.sleep(0.3)

    def wait_for_sensors(self):
        self.get_logger().info("Waiting for /odom and /lidar...")
        while rclpy.ok() and (self.x is None or self.yaw is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Sensors ready.")

    def go_to(self, gx, gy):
        self.get_logger().info(f"Going to waypoint x={gx:.2f}, y={gy:.2f}")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

            dx = gx - self.x
            dy = gy - self.y
            dist = math.hypot(dx, dy)

            if dist < 0.35:
                self.stop()
                return True

            target_yaw = math.atan2(dy, dx)
            yaw_err = wrap(target_yaw - self.yaw)

            # If facing wrong way, rotate first.
            if abs(yaw_err) > 0.45:
                vx = 0.0
                wz = max(min(1.0 * yaw_err, 0.65), -0.65)
            else:
                # Obstacle safety: stop instead of pushing into objects.
                if self.front < 0.75:
                    self.get_logger().warn("Obstacle too close. Stopping and skipping this waypoint.")
                    self.stop()
                    return False

                vx = min(0.65, 0.25 + 0.25 * dist)
                wz = max(min(1.2 * yaw_err, 0.45), -0.45)

            self.cmd(vx, wz)
            time.sleep(0.08)

    def run(self):
        self.wait_for_sensors()

        self.get_logger().info("Starting odom sweep exploration.")

        for wp in self.waypoints:
            self.go_to(*wp)
            time.sleep(0.5)

        self.stop()
        self.get_logger().info("Odom sweep exploration complete.")


def main():
    rclpy.init()
    node = OdomSweepExplorer()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()