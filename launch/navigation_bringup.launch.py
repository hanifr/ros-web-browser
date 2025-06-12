# name: navigation_bringup.launch.py
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
    nav_params_file = LaunchConfiguration('nav_params_file')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_nav_params_file = DeclareLaunchArgument(
        'nav_params_file',
        default_value='/opt/config/nav2_params.yaml',
        description='Full path to the ROS2 parameters file to use for Nav2'
    )

    # Nav2 Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': [
                'controller_server',
                'planner_server',
                'recoveries_server',
                'bt_navigator'
            ]},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Nav2 Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav_params_file, {'use_sim_time': use_sim_time}]
    )

    # Nav2 Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav_params_file, {'use_sim_time': use_sim_time}]
    )

    # Nav2 Recoveries Server
    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[nav_params_file, {'use_sim_time': use_sim_time}]
    )

    # Nav2 Behavior Tree Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav_params_file, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_nav_params_file,
        lifecycle_manager,
        controller_server,
        planner_server,
        recoveries_server,
        bt_navigator,
    ])