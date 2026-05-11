from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('amr_demo')

    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'world.sdf'])
    map_path = PathJoinSubstitution([pkg_share, 'maps', 'town_map_best.yaml'])
    params_path = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])

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

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[params_path],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_path],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
        ],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_path],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_path],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_path],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
        ],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_path],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_path],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_path],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[params_path],
    )

    return LaunchDescription([
        gazebo,
        TimerAction(period=3.0, actions=[bridge, static_lidar_tf]),
        TimerAction(period=6.0, actions=[
            map_server,
            amcl,
            lifecycle_manager_localization,
        ]),
        TimerAction(period=9.0, actions=[
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            lifecycle_manager_navigation,
        ]),
    ])
