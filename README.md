# EECE 698 Autonomous Service Robot Project

Final project repository for EECE 698 by Gabriel Menassa and Larissa Azar.

The project runs an autonomous service robot in a simulated town. The robot first maps the town and records QR landmarks, then uses the saved map and landmark file to execute GUI-selected missions with Nav2 while two traffic robots move in the environment.

## Repository Structure

```text
amr_project_ws/
  src/amr_demo/
    amr_demo/        Python ROS 2 nodes
    launch/          Gazebo, Nav2, AMCL, and traffic launch files
    config/          Nav2 params, SLAM params, missions, landmarks
    maps/            Saved town map files
    worlds/          Gazebo town world
    textures/        QR codes and labels used in the world
    scripts/         Helper scripts for Gazebo, bridges, and exploration
```

Important nodes:

- `mission_gui`: PyQt GUI for selecting mission type and house.
- `run_mission`: sends Nav2 goals for dock, pickup location, house, and back to dock.
- `qr_landmark_detector`: detects QR codes from the robot camera and saves landmark poses.
- `traffic_random_nav_goals`: sends random Nav2 goals to the two traffic robots.
- `traffic_controller`: older/simple loop controller for traffic robots.

## Main Features / Requirements Covered

- Gazebo simulated town with service robot, houses, delivery locations, QR signs, and traffic robots.
- SLAM-based map generation for the town.
- QR landmark detection from the front camera and storage of landmark poses.
- Nav2 navigation on the saved map using AMCL localization.
- Programmatic mission execution without RViz goal clicks.
- GUI mission selection for grocery, food, fire, and medical missions.
- Mission sequence: dock -> pickup landmark -> selected house -> dock.
- Two moving traffic robots using their own Nav2 stacks in the final run.
- 2D LiDAR is used for mapping, localization, obstacle detection, and navigation. The camera is only used for QR landmark detection.

## Setup and Build

These commands assume the repo is at:

```bash
~/EECE698/PROJECT/amr_project_ws
```

Build the package:

```bash
conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
source /opt/ros/jazzy/setup.bash
cd ~/EECE698/PROJECT/amr_project_ws
colcon build --packages-select amr_demo
source install/setup.bash
```

If old Gazebo or ROS processes are still running:

```bash
pkill -9 -f "gz sim" || true
pkill -9 -f "gzserver" || true
pkill -9 -f "gzclient" || true
pkill -9 -f "ruby" || true
pkill -9 -f "ros2" || true
pkill -9 -f "rviz2" || true
pkill -9 -f "parameter_bridge" || true
ros2 daemon stop
sleep 2
ros2 daemon start
```

## Final End-to-End Run

Use separate terminals.

### Terminal 1: Blue Robot + Gazebo + Main Nav2

```bash
conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash
cd ~/EECE698/PROJECT/amr_project_ws
ros2 launch amr_demo nav2_navigation.launch.py
```

### Terminal 2: Traffic Robots

```bash
conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash
cd ~/EECE698/PROJECT/amr_project_ws
ros2 launch amr_demo traffic_nav2.launch.py
```

### Terminal 3: GUI

```bash
conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml
source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash
ros2 run amr_demo mission_gui
```

The GUI runs `run_mission` internally after selecting the mission type and house.

## Run GUI Only

```bash
cd /home/test/EECE698/PROJECT/amr_project_ws
colcon build --packages-select amr_demo
source install/setup.bash
ros2 run amr_demo mission_gui
```

## QR Landmark Detection

Start the main Gazebo/Nav2 system first. Then run the detector in another terminal:

```bash
cd ~/EECE698/PROJECT/amr_project_ws
colcon build --packages-select amr_demo
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash
pkill -f qr_landmark_detector
ros2 run amr_demo qr_landmark_detector
```

Helper teleop command while scanning QR codes:

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Helper camera viewer:

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

To reset landmarks before scanning:

```bash
cd ~/EECE698/PROJECT/amr_project_ws/src/amr_demo
cat > config/landmarks.yaml <<'EOF'
landmarks:
  DOCK: null
  PHARMACY: null
  FIRE_STATION: null
  SUPERMARKET: null
  RESTAURANT: null
  HOUSE_1: null
  HOUSE_2: null
  HOUSE_3: null
  HOUSE_4: null
  HOUSE_5: null
EOF
```

## Mapping / SLAM Requirement

This is the Part 1 mapping workflow used to generate and save the town map. The final mission run uses the saved map in `src/amr_demo/maps/town_map_best.yaml`.

### Terminal 1: Gazebo

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash
cd ~/EECE698/PROJECT/amr_project_ws
bash src/amr_demo/scripts/start_gazebo.sh
```

### Terminal 2: Bridges

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash
cd ~/EECE698/PROJECT/amr_project_ws
bash src/amr_demo/scripts/bridge_all.sh
```

### Terminal 3: Static LiDAR TF

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher \
0.8 0 0.5 0 0 0 \
vehicle_blue/chassis \
vehicle_blue/chassis/gpu_lidar
```

### Terminal 4: SLAM Toolbox

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
use_sim_time:=true \
slam_params_file:=/home/test/EECE698/PROJECT/amr_project_ws/src/amr_demo/config/slam_toolbox.yaml
```

### Terminal 5: Explore the Town

Autonomous sweep:

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash
cd ~/EECE698/PROJECT/amr_project_ws
python3 src/amr_demo/scripts/odom_sweep_explorer.py
```

Teleop was also used during testing/debugging:

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

### Terminal 6: Save Map

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/EECE698/PROJECT/amr_project_ws/install/setup.bash
cd ~/EECE698/PROJECT/amr_project_ws/src/amr_demo/maps
ros2 run nav2_map_server map_saver_cli -f town_map_teleop
```

## Saved Map and Landmark Files

- Current map used by launch files: `src/amr_demo/maps/town_map_best.yaml` and `src/amr_demo/maps/town_map_best.pgm`.
- Map generated from SLAM command above: `src/amr_demo/maps/town_map_teleop.yaml` and `src/amr_demo/maps/town_map_teleop.pgm`.
- Landmark database: `src/amr_demo/config/landmarks.yaml`.
- Mission mapping: `src/amr_demo/config/missions.yaml`.
- If `landmarks.yaml` is changed or regenerated, rebuild with `colcon build --packages-select amr_demo` and source `install/setup.bash` before running missions.
