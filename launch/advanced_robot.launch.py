#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
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
    
    enable_navigation_arg = DeclareLaunchArgument(
        'enable_navigation',
        default_value='false',
        description='Enable navigation stack'
    )
    
    enable_mapping_arg = DeclareLaunchArgument(
        'enable_mapping',
        default_value='false',
        description='Enable SLAM mapping'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_mapping = LaunchConfiguration('enable_mapping')
    
    # Virtual robot node with enhanced features
    virtual_robot_node = Node(
        package='demo_nodes_py',
        executable='virtual_robot.py',
        name='virtual_robot',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_name': robot_name,
            'enable_obstacles': True,
            'obstacle_count': 5,
            'enable_noise': True,
            'sensor_noise_stddev': 0.02,
            'update_rate': 20.0,
            'sensor_rate': 10.0
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
            ('/scan', '/scan')
        ]
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open('/opt/robots/robot.urdf').read()
        }]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # ROS bridge with enhanced configuration
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0',
            'retry_startup_delay': 5,
            'fragment_timeout': 600,
            'delay_between_messages': 0,
            'max_message_size': None,
            'unregister_timeout': 10.0
        }]
    )
    
    # TF2 web republisher for coordinate transforms
    tf2_web_republisher = Node(
        package='tf2_web_republisher',
        executable='tf2_web_republisher',
        name='tf2_web_republisher',
        output='screen',
        parameters=[{
            'angular_threshold': 0.01,
            'translational_threshold': 0.01,
            'rate': 10.0
        }]
    )
    
    # Web video server (for camera streams)
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{
            'port': 8080,
            'server_threads': 1,
            'ros_threads': 2,
            'default_stream_type': 'mjpeg'
        }],
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )
    
    # Autonomous behavior node
    autonomous_behavior = Node(
        package='demo_nodes_py',
        executable='autonomous_behavior.py',
        name='autonomous_behavior',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'behavior_mode': 'obstacle_avoidance',
            'max_linear_speed': 0.5,
            'max_angular_speed': 1.0,
            'obstacle_threshold': 0.5,
            'goal_tolerance': 0.1
        }]
    )
    
    # Diagnostic aggregator
    diagnostic_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        output='screen',
        parameters=[{
            'analyzers.robot.type': 'diagnostic_aggregator/GenericAnalyzer',
            'analyzers.robot.path': 'Robot',
            'analyzers.robot.find_and_remove_prefix': ['virtual_robot'],
            'analyzers.sensors.type': 'diagnostic_aggregator/GenericAnalyzer',
            'analyzers.sensors.path': 'Sensors',
            'analyzers.sensors.find_and_remove_prefix': ['laser', 'camera', 'imu']
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        enable_navigation_arg,
        enable_mapping_arg,
        
        # Core robot nodes
        virtual_robot_node,
        robot_state_publisher,
        joint_state_publisher,
        
        # Web interface nodes
        rosbridge_server,
        tf2_web_republisher,
        
        # Enhanced features (with delay)
        TimerAction(
            period=3.0,
            actions=[
                autonomous_behavior,
                diagnostic_aggregator
            ]
        )
    ])