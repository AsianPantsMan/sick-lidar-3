from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    rviz_config_dir = os.path.join(
            get_package_share_directory('retail_assistant_bringup'),
            'rviz',
            'rplidar_ros.rviz')
    
    #list of nodes or programs to launch
    return  LaunchDescription([
        Node(
            package= "rplidar_ros",#rplidar or our lidar driver package
            executable="rplidar_composition",#actauall node to run
            name="rplidar",
            output="screen",#sends log output to terminal
            parameters=[{'channel_type':'serial',
                         "serial_port" : "/dev/ttyUSB0",
                         'serial_baudrate':460800,
                         "frame_id": 'laser',
                         'inverted':False,
                         'angle_compensate':True,
                         'scan_mode': 'Standard'}],# the first param tells which port to speak to LiDAR and the second paramerter the corndinate frame name or the rpLiDAR coordinate frame 
                  ),
        Node(
            package="slam_toolbox",# The package we are using to implement SLAM
            executable="sync_slam_toolbox_node",#the exectuable within the package
            name= "slam_toolbox",
            output="screen",
            parameters=[{"use_sim_time": False}]#Dont use simlation gazebo time/clock
        ),
         Node(
            package="tf2_ros",#package that handles coordinate transforms
            executable="static_transform_publisher",#executable within the package
            name='odom_tf_publisher',#name of node
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']# first 3 0s are x,y,z translation, next 3 0s are roll pitch yaw rotation, then the parent frame and the child frame
        ),   
        Node(
            package="tf2_ros",#package that handles coordinate transforms
            executable="static_transform_publisher",#executable within the package
            name='lidar_tf_publisher',#name of node
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']# first 3 0s are x,y,z translation, next 3 0s are roll pitch yaw rotation, then the parent frame and the child frame
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=['-d', rviz_config_dir],
            output="screen"
        ),
    ])
