import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='retail_assistant_bringup' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the ros_ign_gazebo package

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world')
    
    world= LaunchConfiguration('world')

    world_arg=DeclareLaunchArgument('world',
                              default_value=default_world,
                              description='SDF world file to load')
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_ign_gazebo'),'launch','ign_gazebo.launch.py')]),
                    launch_arguments={'ign_args':['-r ' , world], 'on_exit_shutdown': 'true'}.items()            #os.path.join(get_package_share_directory(package_name),'worlds','fake_store_layout')
             )

    # Run the spawner node from the ros_ign_gazebo package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_ign_gazebo', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'retail_assistant',
                                   '-z', '0.1'],
                        output='screen')

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','ignition_bridge.yaml')
    ign_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    # Launch them all!
    return LaunchDescription([
        rsp,
        world_arg,
        gazebo,
        spawn_entity,
        ign_bridge,
    ])