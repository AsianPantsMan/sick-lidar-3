import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'retail_assistant_bringup'

    # Robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
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

    # SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'params_file': os.path.join(
                get_package_share_directory(package_name),
                'config',
                'mapper_params_online_async.yaml'
            ),
            'use_sim_time': 'false'
        }.items()
    )

    # Nav2
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
            'params_file': os.path.join(
                get_package_share_directory(package_name),
                'config',
                'nav2_params.yaml'
            )
        }.items()
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
                get_package_share_directory(package_name),
                'config',
                'ekf.yaml'
            )
        ]
    )

    # Delay SLAM and Nav2 so TF / odom / IMU can settle first
    delayed_slam = TimerAction(
        period=3.0,
        actions=[slam]
    )

    delayed_nav2 = TimerAction(
        period=5.0,
        actions=[nav2]
    )

    return LaunchDescription([
        rsp,
        stm32,
        mcu_to_pi,
        sllidar,
        stm_motor,

        # ekf,  # uncomment if you want EKF running

        delayed_slam,
        delayed_nav2,
    ])