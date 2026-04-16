import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'retail_assistant_bringup'
    package_share = get_package_share_directory(package_name)

    nav2_params_file = os.path.join(
        package_share,
        'config',
        'nav2_params.yaml'
    )

    map_yaml_file = '/home/retail-assistant/SLAM/src/retail_assistant_bringup/Slam_maps/auto_map.yaml'

    # Robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                'launch',
                'rsp.py'
            )
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # STM32 sensor node
    stm32 = Node(
        package='retail_assistant_bringup',
        executable='stm32',
        name='stm32_node',
        output='screen',
    )

    # Odom / TF publisher node
    mcu_to_pi = Node(
        package='retail_assistant_bringup',
        executable='mcu_to_pi',
        name='mcu_to_pi_node',
        output='screen',
    )

    # Lidar
    sllidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_c1_launch.py',
            )
        )
    )
    ultrasonic_front = Node(
        package='retail_assistant_bringup',
        executable='ultrasonic_front',
        name='ultrasonic_front_node',
        output='screen',
    )

    ultrasonic_back = Node(
        package='retail_assistant_bringup',
        executable='ultrasonic_back',
        name='ultrasonic_back_node',
        output='screen',
    )
    # Motor node
    stm_motor = Node(
        package='retail_assistant_bringup',
        executable='stm_m',
        name='stm_m_node',
        output='screen',
    )

    # Optional EKF node
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(
                package_share,
                'config',
                'ekf.yaml'
            ),
            {'use_sim_time': False}
        ]
    )

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'yaml_filename': map_yaml_file}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params_file,

            # Explicit overrides here if you want them local to launch:
            {'use_sim_time': False},
            {'base_frame_id': 'base_link'},
            {'odom_frame_id': 'odom'},
            {'global_frame_id': 'map'},
            {'scan_topic': '/scan'},

            # Optional fixed startup pose:
            # set to True if you want AMCL to start from this pose automatically
            {'set_initial_pose': True},
            {'always_reset_initial_pose': False},
            {'initial_pose.x': 0.0},
            {'initial_pose.y': 0.0},
            {'initial_pose.yaw': 0.0},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Lifecycle manager for map_server + amcl
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    # Nav2 navigation stack
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file
        }.items()
    )

    navigation = Node(
        package='retail_assistant_bringup',
        executable='navigation',
        name='navigation_node',
        output='screen',
    )

    delayed_localization = TimerAction(
        period=3.0,
        actions=[map_server, amcl, lifecycle_manager_localization]
    )

    delayed_nav2 = TimerAction(
        period=10.0,
        actions=[nav2]
    )

    delayed_navigation = TimerAction(
        period=20.0,
        actions=[navigation]
    )

    return LaunchDescription([
        rsp,
        stm32,
        mcu_to_pi,
        sllidar,
        stm_motor,

        # ekf,  # uncomment if you want EKF running

        delayed_localization,
        delayed_nav2,
        # delayed_navigation
    ])