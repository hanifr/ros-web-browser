#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String, Header
import math
import time
import random
import numpy as np

class VirtualRobot(Node):
    def __init__(self):
        super().__init__('virtual_robot')
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.max_linear_vel = 2.0
        self.max_angular_vel = 2.0
        
        # Robot parameters
        self.wheel_base = 0.5  # Distance between wheels
        self.robot_radius = 0.3
        
        # Environment obstacles (simple circles)
        self.obstacles = [
            {'x': 3.0, 'y': 2.0, 'radius': 0.5},
            {'x': -2.0, 'y': -1.0, 'radius': 0.3},
            {'x': 1.0, 'y': -3.0, 'radius': 0.4},
        ]
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.vel_echo_pub = self.create_publisher(Twist, '/cmd_vel_echo', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.range_pub = self.create_publisher(Range, '/range_sensor', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.mode_sub = self.create_subscription(
            String, '/robot_mode', self.mode_callback, 10)
        self.reset_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.reset_callback, 10)
        
        # Timers
        self.update_timer = self.create_timer(0.05, self.update_robot)  # 20 Hz
        self.sensor_timer = self.create_timer(0.1, self.publish_sensors)  # 10 Hz
        
        self.get_logger().info('Virtual robot started')
        self.get_logger().info(f'Robot position: ({self.x:.2f}, {self.y:.2f})')
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        # Limit velocities
        self.linear_vel = max(-self.max_linear_vel, 
                             min(self.max_linear_vel, msg.linear.x))
        self.angular_vel = max(-self.max_angular_vel, 
                              min(self.max_angular_vel, msg.angular.z))
        
        # Echo the command for visualization
        echo_msg = Twist()
        echo_msg.linear.x = self.linear_vel
        echo_msg.angular.z = self.angular_vel
        self.vel_echo_pub.publish(echo_msg)

        # ADD THIS TOO
        self.get_logger().info(f'🤖 Robot velocities set: linear={self.linear_vel:.2f}, angular={self.angular_vel:.2f}')
    
    def mode_callback(self, msg):
        """Handle robot mode changes"""
        mode = msg.data
        self.get_logger().info(f'Robot mode changed to: {mode}')
        
        if mode == 'stop':
            self.linear_vel = 0.0
            self.angular_vel = 0.0
        elif mode == 'reset':
            self.reset_robot()
    
    def reset_callback(self, msg):
        """Handle reset pose commands"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Convert quaternion to euler angle
        orientation = msg.pose.pose.orientation
        self.theta = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )
        
        self.get_logger().info(f'Robot reset to: ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.theta):.1f}°)')
    
    def reset_robot(self):
        """Reset robot to origin"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
    
    def update_robot(self):
        """Update robot physics simulation"""
        dt = 0.05  # 20 Hz update rate
        
        # Simple differential drive kinematics
        if abs(self.angular_vel) < 1e-6:
            # Straight line motion
            self.x += self.linear_vel * math.cos(self.theta) * dt
            self.y += self.linear_vel * math.sin(self.theta) * dt
        else:
            # Curved motion
            radius = self.linear_vel / self.angular_vel
            
            # Instantaneous center of curvature
            icc_x = self.x - radius * math.sin(self.theta)
            icc_y = self.y + radius * math.cos(self.theta)
            
            # Update position and orientation
            dtheta = self.angular_vel * dt
            self.theta += dtheta
            
            # Rotate around ICC
            cos_dtheta = math.cos(dtheta)
            sin_dtheta = math.sin(dtheta)
            
            dx = self.x - icc_x
            dy = self.y - icc_y
            
            self.x = icc_x + dx * cos_dtheta - dy * sin_dtheta
            self.y = icc_y + dx * sin_dtheta + dy * cos_dtheta
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        
        # Publish pose
        self.publish_pose()
        self.publish_odometry()
    
    def publish_pose(self):
        """Publish robot pose"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0
        
        # Convert angle to quaternion
        pose_msg.pose.orientation.z = math.sin(self.theta / 2)
        pose_msg.pose.orientation.w = math.cos(self.theta / 2)
        
        self.pose_pub.publish(pose_msg)
    
    def publish_odometry(self):
        """Publish odometry"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)
        
        # Velocity
        odom_msg.twist.twist.linear.x = self.linear_vel
        odom_msg.twist.twist.angular.z = self.angular_vel
        
        # Add some noise to make it realistic
        position_noise = 0.01
        odom_msg.pose.pose.position.x += random.gauss(0, position_noise)
        odom_msg.pose.pose.position.y += random.gauss(0, position_noise)
        
        self.odom_pub.publish(odom_msg)
    
    def publish_sensors(self):
        """Publish sensor data"""
        self.publish_laser_scan()
        self.publish_range_sensor()
    
    def publish_laser_scan(self):
        """Simulate and publish laser scan"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'
        
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = math.pi / 180.0  # 1 degree
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = 10.0
        
        # Generate scan points
        num_points = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
        ranges = []
        
        for i in range(num_points):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            global_angle = self.theta + angle
            
            # Default max range
            min_range = scan_msg.range_max
            
            # Check obstacles
            for obs in self.obstacles:
                # Vector from robot to obstacle
                dx = obs['x'] - self.x
                dy = obs['y'] - self.y
                dist_to_center = math.sqrt(dx*dx + dy*dy)
                
                # Angle to obstacle center
                angle_to_obs = math.atan2(dy, dx)
                angle_diff = math.atan2(math.sin(angle_to_obs - global_angle), 
                                      math.cos(angle_to_obs - global_angle))
                
                # Check if ray intersects obstacle
                if abs(angle_diff) < math.atan(obs['radius'] / max(dist_to_center, 0.1)):
                    # Approximate range to obstacle surface
                    range_to_obs = max(0.1, dist_to_center - obs['radius'])
                    min_range = min(min_range, range_to_obs)
            
            # Add some noise
            min_range += random.gauss(0, 0.02)
            ranges.append(max(scan_msg.range_min, min(scan_msg.range_max, min_range)))
        
        scan_msg.ranges = ranges
        scan_msg.intensities = [100.0] * len(ranges)  # Constant intensity
        
        self.scan_pub.publish(scan_msg)
    
    def publish_range_sensor(self):
        """Publish simple range sensor data"""
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = 'range_sensor_frame'
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.26  # ~15 degrees
        range_msg.min_range = 0.02
        range_msg.max_range = 4.0
        
        # Simple forward-looking range sensor
        min_range = range_msg.max_range
        
        for obs in self.obstacles:
            dx = obs['x'] - self.x
            dy = obs['y'] - self.y
            
            # Check if obstacle is in front
            angle_to_obs = math.atan2(dy, dx)
            angle_diff = math.atan2(math.sin(angle_to_obs - self.theta), 
                                  math.cos(angle_to_obs - self.theta))
            
            if abs(angle_diff) < range_msg.field_of_view / 2:
                dist = math.sqrt(dx*dx + dy*dy) - obs['radius']
                min_range = min(min_range, max(range_msg.min_range, dist))
        
        range_msg.range = min_range
        self.range_pub.publish(range_msg)

def main(args=None):
    rclpy.init(args=args)
    robot = VirtualRobot()
    
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()