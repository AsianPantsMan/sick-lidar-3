import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    package_name='retail_assistant_bringup' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )
    mcu_to_pi = Node(
    package="retail_assistant_bringup",
    executable="mcu_to_pi",   # if installed as a program
    name="mcu_to_pi_node",
    output="screen",
)
    sllidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sllidar_ros2"),
                "launch",
                "sllidar_c1_launch.py",
            )
        ),
        launch_arguments={
        }.items(),
    )
    slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join
                                           (get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py')]),# fix _local later
                                          launch_arguments={'params_file': os.path.join(get_package_share_directory(package_name),'config','mapper_params_online_async.yaml')}.items())
                                          
    # Launch them all!

    return LaunchDescription([
        rsp,
        sllidar,        
        mcu_to_pi,
        slam,
    ])