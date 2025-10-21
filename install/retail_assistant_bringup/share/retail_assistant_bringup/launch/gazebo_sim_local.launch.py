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
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),
                    launch_arguments={'world': os.path.join(get_package_share_directory(package_name),'worlds','fake_store_layout.world')}.items()#os.path.join(get_package_share_directory(package_name),'worlds','fake_store_layout')
             )
    print(os.path.join(get_package_share_directory(package_name),'worlds','fake_store_layout.world'))
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'retail_assistant'],
                        output='screen')

    slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join
                                           (get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py')]),# fix _local later
                                          launch_arguments={'slam_params_file': os.path.join(get_package_share_directory(package_name),'config','mapper_params_online_async_local.yaml')}.items())
    nav2=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join
                                       (get_package_share_directory('nav2_bringup'),'launch','navigation_launch.py')])
                                       ,launch_arguments={'use_sim_time': 'true',
                                                          'params_file': os.path.join(
                                                             get_package_share_directory('nav2_bringup'),
                                                             'params',
                                                             'nav2_params.yaml'
                                                                )}.items())                              
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        slam,
        nav2
    ])