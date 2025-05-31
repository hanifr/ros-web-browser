#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='virtual_robot',
        description='Name of the robot'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    
    # Virtual robot node
    virtual_robot_node = Node(
        package='demo_nodes_py',  # We'll use a simple Python node
        executable='virtual_robot.py',
        name='virtual_robot',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_name': robot_name
        }]
    )
    
    # Rosbridge server
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0'
        }]
    )
    
    # TF2 web republisher (optional, for better web integration)
    tf2_web_republisher = Node(
        package='tf2_web_republisher',
        executable='tf2_web_republisher',
        name='tf2_web_republisher',
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        virtual_robot_node,
        rosbridge_server,
        tf2_web_republisher
    ])