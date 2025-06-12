# slam_bringup.launch.py
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value='/opt/config/slam_toolbox_params.yaml',
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Static transform publisher for map -> odom
    # This will be handled by SLAM Toolbox, but we can add it for initial setup
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        slam_toolbox_node,
        # map_to_odom_tf,  # Commented out as SLAM Toolbox will handle this
    ])