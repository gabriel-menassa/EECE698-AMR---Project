from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('amr_demo')

    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'world.sdf'])
    map_path = PathJoinSubstitution([pkg_share, 'maps', 'town_map_best.yaml'])
    params_path = PathJoinSubstitution([pkg_share, 'config', 'amcl_params.yaml'])

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_path],
        output='screen'
    )

    bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/vehicle_blue/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/world/car_world/model/vehicle_blue/link/chassis/sensor/front_cam/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/world/car_world/model/vehicle_blue/link/chassis/sensor/front_cam/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args',
            '-r', '/model/vehicle_blue/odometry:=/odom',
            '-r', '/model/vehicle_blue/tf:=/tf',
            '-r', '/world/car_world/model/vehicle_blue/link/chassis/sensor/front_cam/image:=/camera/image_raw',
            '-r', '/world/car_world/model/vehicle_blue/link/chassis/sensor/front_cam/camera_info:=/camera/camera_info',
        ],
        output='screen'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_path,
            {'yaml_filename': map_path},
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_path],
    )

    static_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_lidar_tf',
        arguments=[
            '0.8', '0', '0.5',
            '0', '0', '0',
            'vehicle_blue/chassis',
            'vehicle_blue/chassis/gpu_lidar',
        ],
        output='screen',
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[params_path],
    )

    return LaunchDescription([
        gazebo,
        TimerAction(period=3.0, actions=[bridge, static_lidar_tf]),
        TimerAction(period=6.0, actions=[map_server, amcl, lifecycle_manager]),
    ])
