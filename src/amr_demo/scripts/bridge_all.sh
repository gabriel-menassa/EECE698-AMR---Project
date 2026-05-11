#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /model/vehicle_blue/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V \
  /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /world/car_world/model/vehicle_blue/link/chassis/sensor/front_cam/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /world/car_world/model/vehicle_blue/link/chassis/sensor/front_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  --ros-args \
  -r /model/vehicle_blue/odometry:=/odom \
  -r /model/vehicle_blue/tf:=/tf \
  -r /world/car_world/model/vehicle_blue/link/chassis/sensor/front_cam/image:=/camera/image_raw \
  -r /world/car_world/model/vehicle_blue/link/chassis/sensor/front_cam/camera_info:=/camera/camera_info
