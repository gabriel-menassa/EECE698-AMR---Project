#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class LidarExplorer(Node):
    def __init__(self):
        super().__init__("lidar_explorer")

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(LaserScan, "/lidar", self.scan_cb, 10)

        self.front = float("inf")
        self.front_left = float("inf")
        self.front_right = float("inf")
        self.left = float("inf")
        self.right = float("inf")

        self.start_time = time.time()
        self.duration = 360.0

        self.last_escape_time = 0.0
        self.escape_until = 0.0
        self.escape_direction = 1.0
        self.escape_phase = "none"
        self.escape_back_until = 0.0
        self.escape_turn_until = 0.0

    def sector_min(self, ranges, start_frac, end_frac):
        n = len(ranges)
        vals = ranges[int(start_frac * n):int(end_frac * n)]
        vals = [v for v in vals if math.isfinite(v) and 0.05 < v < 20.0]
        return min(vals) if vals else float("inf")

    def scan_cb(self, msg):
        ranges = list(msg.ranges)

        self.right = self.sector_min(ranges, 0.05, 0.25)
        self.front_right = self.sector_min(ranges, 0.30, 0.45)
        self.front = self.sector_min(ranges, 0.45, 0.55)
        self.front_left = self.sector_min(ranges, 0.55, 0.70)
        self.left = self.sector_min(ranges, 0.75, 0.95)

    def publish_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)

    def run(self):
        self.get_logger().info("Starting improved LiDAR autonomous exploration.")

        while rclpy.ok() and time.time() - self.start_time < self.duration:
            rclpy.spin_once(self, timeout_sec=0.02)

            now = time.time()

            # Escape behavior if boxed in or too close to obstacle.
            # Two-phase escape:
            # 1) reverse straight to detach from obstacle
            # 2) turn away from obstacle
            if now < self.escape_back_until:
                self.publish_cmd(-0.65, 0.0)
                time.sleep(0.1)
                continue

            if now < self.escape_turn_until:
                self.publish_cmd(-0.10, self.escape_direction * 1.00)
                time.sleep(0.1)
                continue

            # If obstacle ahead, turn toward more open side.
            if self.front < 1.15 or self.front_left < 0.80 or self.front_right < 0.80:
                self.escape_direction = 1.0 if self.left > self.right else -1.0

                # If very close, back up harder and longer.
                if self.front < 0.55 or self.front_left < 0.45 or self.front_right < 0.45:
                    self.escape_back_until = now + 1.6
                    self.escape_turn_until = now + 2.5
                else:
                    self.escape_back_until = now + 0.9
                    self.escape_turn_until = now + 1.6

                self.publish_cmd(-0.65, 0.0)
                time.sleep(0.1)
                continue

            # Prefer loose right-wall following.
            desired_right = 1.2
            error = desired_right - self.right

            # If no wall nearby, wander left/right slowly to discover space.
            if self.right == float("inf") or self.right > 3.0:
                vx = 0.55
                wz = 0.30 * math.sin(now * 0.45)
            else:
                vx = 0.55
                wz = 0.25 * error
                wz = max(min(wz, 0.35), -0.35)

            self.publish_cmd(vx, wz)
            time.sleep(0.1)

        self.stop()
        self.get_logger().info("Exploration complete.")

    def stop(self):
        self.publish_cmd(0.0, 0.0)
        time.sleep(0.5)


def main():
    rclpy.init()
    node = LidarExplorer()

    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()