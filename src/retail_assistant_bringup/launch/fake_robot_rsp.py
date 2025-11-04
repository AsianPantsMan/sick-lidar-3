import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    namespace = LaunchConfiguration('namespace')   # <--- NEW LINE

    # Locate your fake robot xacro
    pkg_path = os.path.join(get_package_share_directory('retail_assistant_bringup'))
    xacro_file = os.path.join(pkg_path, 'description', 'fake_person_robot.xacro')

    # Generate URDF
    robot_description_config = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=', use_sim_time
    ])
    robot_description = ParameterValue(robot_description_config, value_type=str)

    # Parameters
    params = {
        'robot_description': robot_description,
        'use_sim_time': use_sim_time
    }

    # Node — now inside namespace
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,   # <--- NEW LINE
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='fake_robot', description='Namespace for this robot'),  # <--- NEW LINE
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_ros2_control', default_value='true'),
        node_robot_state_publisher
    ])
