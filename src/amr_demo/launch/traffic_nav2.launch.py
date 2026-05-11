from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def nav2_nodes(robot_name, params_path, map_path):
    ns = robot_name
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_path,
            root_key=robot_name,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    common_remaps = [
        ('/tf', '/tf'),
        ('/tf_static', '/tf_static'),
    ]

    localization_nodes = ['map_server', 'amcl']

    navigation_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    amcl_params = {
        'use_sim_time': True,
        'global_frame_id': 'map',
        'odom_frame_id': f'{robot_name}/odom',
        'base_frame_id': f'{robot_name}/chassis',
        'scan_topic': f'/{robot_name}/lidar',
        'tf_broadcast': True,
    }

    controller_params = {
        'use_sim_time': True,
        'odom_topic': f'/{robot_name}/odom',
        'controller_plugins': ['FollowPath'],

        'FollowPath.plugin': 'dwb_core::DWBLocalPlanner',
        'FollowPath.debug_trajectory_details': True,
        'FollowPath.min_vel_x': 0.0,
        'FollowPath.min_vel_y': 0.0,
        'FollowPath.max_vel_x': 0.70,
        'FollowPath.max_vel_y': 0.0,
        'FollowPath.max_vel_theta': 1.6,
        'FollowPath.min_speed_xy': 0.0,
        'FollowPath.max_speed_xy': 0.70,
        'FollowPath.min_speed_theta': 0.0,
        'FollowPath.acc_lim_x': 2.0,
        'FollowPath.acc_lim_y': 0.0,
        'FollowPath.acc_lim_theta': 3.0,
        'FollowPath.decel_lim_x': -2.0,
        'FollowPath.decel_lim_y': 0.0,
        'FollowPath.decel_lim_theta': -3.0,
        'FollowPath.vx_samples': 20,
        'FollowPath.vy_samples': 5,
        'FollowPath.vtheta_samples': 20,
        'FollowPath.sim_time': 1.5,
        'FollowPath.linear_granularity': 0.05,
        'FollowPath.angular_granularity': 0.025,
        'FollowPath.transform_tolerance': 0.2,
        'FollowPath.xy_goal_tolerance': 0.35,
        'FollowPath.trans_stopped_velocity': 0.25,
        'FollowPath.short_circuit_trajectory_evaluation': True,
        'FollowPath.stateful': True,
        'FollowPath.critics': [
            'RotateToGoal',
            'Oscillation',
            'BaseObstacle',
            'GoalAlign',
            'PathAlign',
            'PathDist',
            'GoalDist',
        ],
        'FollowPath.BaseObstacle.scale': 0.02,
        'FollowPath.PathAlign.scale': 32.0,
        'FollowPath.PathAlign.forward_point_distance': 0.1,
        'FollowPath.GoalAlign.scale': 24.0,
        'FollowPath.GoalAlign.forward_point_distance': 0.1,
        'FollowPath.PathDist.scale': 32.0,
        'FollowPath.GoalDist.scale': 24.0,
        'FollowPath.RotateToGoal.scale': 32.0,
        'FollowPath.RotateToGoal.slowing_factor': 5.0,
        'FollowPath.RotateToGoal.lookahead_time': -1.0,
    }

    velocity_smoother_params = {
        'use_sim_time': True,
        'odom_topic': f'/{robot_name}/odom',
        'max_velocity': [0.70, 0.0, 1.6],
        'min_velocity': [0.0, 0.0, -1.6],
        'max_accel': [2.0, 0.0, 3.0],
        'max_decel': [-2.0, 0.0, -3.0],
    }

    lifecycle_localization_params = {
        'use_sim_time': True,
        'autostart': True,
        'node_names': localization_nodes,
    }

    lifecycle_navigation_params = {
        'use_sim_time': True,
        'autostart': True,
        'node_names': navigation_nodes,
    }

    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{robot_name}_static_lidar_tf',
            arguments=[
                '0.8', '0', '0.5',
                '0', '0', '0',
                f'{robot_name}/chassis',
                f'{robot_name}/chassis/gpu_lidar',
            ],
            output='screen',
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            namespace=ns,
            name='map_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}, {'yaml_filename': map_path}],
            remappings=common_remaps,
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            namespace=ns,
            name='amcl',
            output='screen',
            parameters=[configured_params, amcl_params],
            remappings=common_remaps,
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            namespace=ns,
            name='controller_server',
            output='screen',
            parameters=[configured_params, controller_params],
            remappings=common_remaps + [('cmd_vel', 'cmd_vel_nav')],
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            namespace=ns,
            name='smoother_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}],
            remappings=common_remaps,
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            namespace=ns,
            name='planner_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}],
            remappings=common_remaps,
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            namespace=ns,
            name='behavior_server',
            output='screen',
            parameters=[
                configured_params,
                {
                    'use_sim_time': True,
                    'global_frame': f'{robot_name}/odom',
                    'robot_base_frame': f'{robot_name}/chassis',
                },
            ],
            remappings=common_remaps + [('cmd_vel', 'cmd_vel_nav')],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            namespace=ns,
            name='bt_navigator',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}],
            remappings=common_remaps,
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            namespace=ns,
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params, {'use_sim_time': True}],
            remappings=common_remaps,
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            namespace=ns,
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params, velocity_smoother_params],
            remappings=common_remaps + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace=ns,
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[lifecycle_localization_params],
            remappings=common_remaps,
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace=ns,
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[lifecycle_navigation_params],
            remappings=common_remaps,
        ),
    ]


def generate_launch_description():
    pkg_share = FindPackageShare('amr_demo')

    map_path = PathJoinSubstitution([pkg_share, 'maps', 'town_map_best.yaml'])
    params_1 = PathJoinSubstitution([pkg_share, 'config', 'nav2_params_traffic_1.yaml'])
    params_2 = PathJoinSubstitution([pkg_share, 'config', 'nav2_params_traffic_2.yaml'])

    traffic_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/robot_traffic_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/robot_traffic_2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/robot_traffic_1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/robot_traffic_2/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/robot_traffic_1/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/model/robot_traffic_2/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/robot_traffic_1/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/robot_traffic_2/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '--ros-args',
            '-r', '/model/robot_traffic_1/odometry:=/robot_traffic_1/odom',
            '-r', '/model/robot_traffic_2/odometry:=/robot_traffic_2/odom',
            '-r', '/model/robot_traffic_1/tf:=/tf',
            '-r', '/model/robot_traffic_2/tf:=/tf',
        ],
        output='screen',
    )

    traffic_random_goals = Node(
        package='amr_demo',
        executable='traffic_random_nav_goals',
        name='traffic_random_nav_goals',
        output='screen',
        arguments=[
            '--goal-delay', '2.0',
            '--goal-timeout', '120.0',
            '--initial-pose-repeats', '6',
            '--initial-pose-period', '0.5',
            '--start-delay', '3.0',
        ],
    )

    return LaunchDescription([
        traffic_bridge,
        TimerAction(period=3.0, actions=nav2_nodes('robot_traffic_1', params_1, map_path)),
        TimerAction(period=6.0, actions=nav2_nodes('robot_traffic_2', params_2, map_path)),
        TimerAction(period=12.0, actions=[traffic_random_goals]),
    ])
