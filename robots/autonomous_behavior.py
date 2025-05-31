#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import numpy as np
from enum import Enum

class BehaviorMode(Enum):
    IDLE = "idle"
    MANUAL = "manual"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    FOLLOW_WALL = "follow_wall"
    EXPLORE = "explore"
    GO_TO_GOAL = "go_to_goal"

class AutonomousBehavior(Node):
    def __init__(self):
        super().__init__('autonomous_behavior')
        
        # Parameters
        self.declare_parameter('behavior_mode', 'obstacle_avoidance')
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('obstacle_threshold', 0.5)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('wall_follow_distance', 0.3)
        self.declare_parameter('exploration_timeout', 30.0)
        
        # State variables
        self.current_mode = BehaviorMode.IDLE
        self.robot_pose = None
        self.laser_data = None
        self.goal_point = None
        self.exploration_start_time = None
        
        # Control parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.wall_follow_distance = self.get_parameter('wall_follow_distance').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # Subscribers
        self.mode_sub = self.create_subscription(
            String, '/behavior_mode', self.mode_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.goal_sub = self.create_subscription(
            Point, '/move_base_simple/goal', self.goal_callback, 10)
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('Autonomous behavior node started')
    
    def mode_callback(self, msg):
        """Handle behavior mode changes"""
        try:
            new_mode = BehaviorMode(msg.data)
            if new_mode != self.current_mode:
                self.get_logger().info(f'Behavior mode changed: {self.current_mode.value} -> {new_mode.value}')
                self.current_mode = new_mode
                
                # Reset state for certain modes
                if new_mode == BehaviorMode.EXPLORE:
                    self.exploration_start_time = self.get_clock().now()
                elif new_mode == BehaviorMode.IDLE:
                    self.stop_robot()
                    
        except ValueError:
            self.get_logger().warn(f'Invalid behavior mode: {msg.data}')
    
    def odom_callback(self, msg):
        """Handle odometry updates"""
        self.robot_pose = msg.pose.pose
    
    def laser_callback(self, msg):
        """Handle laser scan updates"""
        self.laser_data = msg
    
    def goal_callback(self, msg):
        """Handle goal point updates"""
        self.goal_point = msg
        self.current_mode = BehaviorMode.GO_TO_GOAL
        self.get_logger().info(f'New goal received: ({msg.x:.2f}, {msg.y:.2f})')
    
    def control_loop(self):
        """Main control loop"""
        if self.current_mode == BehaviorMode.IDLE:
            self.stop_robot()
        elif self.current_mode == BehaviorMode.OBSTACLE_AVOIDANCE:
            self.obstacle_avoidance_behavior()
        elif self.current_mode == BehaviorMode.FOLLOW_WALL:
            self.wall_following_behavior()
        elif self.current_mode == BehaviorMode.EXPLORE:
            self.exploration_behavior()
        elif self.current_mode == BehaviorMode.GO_TO_GOAL:
            self.go_to_goal_behavior()
        
        # Publish status
        status_msg = String()
        status_msg.data = self.current_mode.value
        self.status_pub.publish(status_msg)
    
    def obstacle_avoidance_behavior(self):
        """Simple obstacle avoidance using laser scan"""
        if not self.laser_data:
            return
        
        cmd = Twist()
        
        # Get front, left, and right ranges
        ranges = np.array(self.laser_data.ranges)
        ranges[ranges == 0] = float('inf')  # Replace 0 with inf
        ranges[ranges > self.laser_data.range_max] = self.laser_data.range_max
        
        # Front sector (±30 degrees)
        front_start = len(ranges) - 30
        front_end = 30
        front_ranges = np.concatenate([ranges[front_start:], ranges[:front_end]])
        min_front_dist = np.min(front_ranges)
        
        # Left sector (60-120 degrees)
        left_ranges = ranges[60:120]
        min_left_dist = np.min(left_ranges)
        
        # Right sector (240-300 degrees)
        right_ranges = ranges[240:300]
        min_right_dist = np.min(right_ranges)
        
        # Decision logic
        if min_front_dist > self.obstacle_threshold:
            # Clear ahead - move forward
            cmd.linear.x = self.max_linear_speed * 0.7
            cmd.angular.z = 0.0
        else:
            # Obstacle ahead - turn
            if min_left_dist > min_right_dist:
                # More space on left - turn left
                cmd.linear.x = 0.1
                cmd.angular.z = self.max_angular_speed * 0.8
            else:
                # More space on right - turn right
                cmd.linear.x = 0.1
                cmd.angular.z = -self.max_angular_speed * 0.8
        
        self.cmd_vel_pub.publish(cmd)
    
    def wall_following_behavior(self):
        """Follow wall on the right side"""
        if not self.laser_data:
            return
        
        cmd = Twist()
        ranges = np.array(self.laser_data.ranges)
        ranges[ranges == 0] = float('inf')
        
        # Right side ranges (270±15 degrees)
        right_ranges = ranges[255:285]
        right_dist = np.min(right_ranges)
        
        # Front ranges
        front_ranges = np.concatenate([ranges[-15:], ranges[:15]])
        front_dist = np.min(front_ranges)
        
        # Wall following logic
        if front_dist < self.obstacle_threshold:
            # Obstacle ahead - turn left
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_angular_speed * 0.5
        elif right_dist > self.wall_follow_distance * 1.5:
            # Too far from wall - turn right
            cmd.linear.x = self.max_linear_speed * 0.5
            cmd.angular.z = -self.max_angular_speed * 0.3
        elif right_dist < self.wall_follow_distance * 0.8:
            # Too close to wall - turn left
            cmd.linear.x = self.max_linear_speed * 0.5
            cmd.angular.z = self.max_angular_speed * 0.3
        else:
            # Good distance - move forward
            cmd.linear.x = self.max_linear_speed * 0.6
            cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)
    
    def exploration_behavior(self):
        """Random exploration with obstacle avoidance"""
        if not self.laser_data:
            return
        
        # Check exploration timeout
        if self.exploration_start_time:
            elapsed = (self.get_clock().now() - self.exploration_start_time).nanoseconds / 1e9
            timeout = self.get_parameter('exploration_timeout').value
            if elapsed > timeout:
                self.current_mode = BehaviorMode.IDLE
                self.get_logger().info('Exploration timeout - switching to idle')
                return
        
        cmd = Twist()
        ranges = np.array(self.laser_data.ranges)
        ranges[ranges == 0] = float('inf')
        
        # Check front for obstacles
        front_ranges = np.concatenate([ranges[-30:], ranges[:30]])
        min_front_dist = np.min(front_ranges)
        
        if min_front_dist > self.obstacle_threshold * 1.5:
            # Clear ahead - move forward with some randomness
            cmd.linear.x = self.max_linear_speed * (0.6 + 0.3 * np.random.random())
            cmd.angular.z = self.max_angular_speed * 0.2 * (np.random.random() - 0.5)
        else:
            # Obstacle ahead - turn randomly
            cmd.linear.x = 0.1
            turn_direction = 1 if np.random.random() > 0.5 else -1
            cmd.angular.z = turn_direction * self.max_angular_speed * 0.8
        
        self.cmd_vel_pub.publish(cmd)
    
    def go_to_goal_behavior(self):
        """Simple goal-seeking behavior"""
        if not self.robot_pose or not self.goal_point:
            return
        
        # Calculate distance and angle to goal
        dx = self.goal_point.x - self.robot_pose.position.x
        dy = self.goal_point.y - self.robot_pose.position.y
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        angle_to_goal = math.atan2(dy, dx)
        
        # Get robot orientation
        orientation = self.robot_pose.orientation
        robot_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )
        
        # Calculate angle difference
        angle_diff = math.atan2(
            math.sin(angle_to_goal - robot_yaw),
            math.cos(angle_to_goal - robot_yaw)
        )
        
        cmd = Twist()
        
        # Check if goal is reached
        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info('Goal reached!')
            self.current_mode = BehaviorMode.IDLE
            self.goal_point = None
            return
        
        # Check for obstacles (simple front check)
        if self.laser_data:
            ranges = np.array(self.laser_data.ranges)
            front_ranges = np.concatenate([ranges[-15:], ranges[:15]])
            min_front_dist = np.min(front_ranges[ranges[front_ranges] > 0])
            
            if min_front_dist < self.obstacle_threshold:
                # Obstacle detected - switch to obstacle avoidance temporarily
                self.obstacle_avoidance_behavior()
                return
        
        # Move towards goal
        if abs(angle_diff) > 0.1:  # Need to turn
            cmd.angular.z = np.sign(angle_diff) * min(abs(angle_diff), self.max_angular_speed * 0.8)
            cmd.linear.x = self.max_linear_speed * 0.3  # Slow forward while turning