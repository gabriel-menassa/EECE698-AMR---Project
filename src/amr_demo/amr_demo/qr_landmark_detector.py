#!/usr/bin/env python3

import math
import os
import yaml

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener


VALID_LANDMARKS = {
    "DOCK",
    "PHARMACY",
    "FIRE_STATION",
    "SUPERMARKET",
    "RESTAURANT",
    "HOUSE_1",
    "HOUSE_2",
    "HOUSE_3",
    "HOUSE_4",
    "HOUSE_5",
}


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class QRLandmarkDetector(Node):
    def __init__(self):
        super().__init__("qr_landmark_detector")

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_cb,
            10,
        )

        self.landmarks_path = (
            "/home/test/EECE698/PROJECT/amr_project_ws/src/amr_demo/config/landmarks.yaml"
        )

        self.seen = set()

        self.get_logger().info("QR landmark detector started.")
        self.get_logger().info(f"Saving landmarks to: {self.landmarks_path}")

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                "vehicle_blue/chassis",
                rclpy.time.Time(),
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = yaw_from_quat(tf.transform.rotation)

            return x, y, yaw

        except Exception as e:
            self.get_logger().warn(f"Could not get map pose yet: {e}")
            return None

    def load_landmarks(self):
        if not os.path.exists(self.landmarks_path):
            return {"landmarks": {name: None for name in sorted(VALID_LANDMARKS)}}

        with open(self.landmarks_path, "r") as f:
            data = yaml.safe_load(f)

        if data is None:
            data = {"landmarks": {}}

        if "landmarks" not in data:
            data["landmarks"] = {}

        for name in VALID_LANDMARKS:
            data["landmarks"].setdefault(name, None)

        return data

    def save_landmark(self, name, x, y, yaw):
        data = self.load_landmarks()

        data["landmarks"][name] = {
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
        }

        with open(self.landmarks_path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)

        self.seen.add(name)

        self.get_logger().info(
            f"Saved {name}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        decoded, points, _ = self.detector.detectAndDecode(frame)

        if not decoded:
            return

        name = decoded.strip().upper()

        if name not in VALID_LANDMARKS:
            self.get_logger().warn(f"Ignoring unknown QR text: {decoded}")
            return

        if name in self.seen:
            return

        # Require QR to be visually large enough before saving.
        # This prevents saving far-away landmarks from bad robot poses.
        if points is None:
            return

        pts = points.reshape(-1, 2)
        area = cv2.contourArea(pts.astype("float32"))

        self.get_logger().info(f"Detected {name}, area={area:.0f}")

        if area < 105000:
            self.get_logger().info(f"Detected {name}, but too far/small. area={area:.0f}")
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        x, y, yaw = pose
        self.save_landmark(name, x, y, yaw)


def main():
    rclpy.init()
    node = QRLandmarkDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


    