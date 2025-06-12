#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math
import numpy as np

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0, 0, 0, 0]
    q[0] = sr * cp * cy - cr * sp * sy  # x
    q[1] = cr * sp * cy + sr * cp * sy  # y
    q[2] = cr * cp * sy - sr * sp * cy  # z
    q[3] = cr * cp * cy + sr * sp * sy  # w
    return q

class EnhancedVirtualRobot(Node):
    def __init__(self):
        super().__init__('enhanced_virtual_robot')
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Robot parameters
        self.wheel_base = 0.5
        self.max_linear_vel = 2.0
        self.max_angular_vel = 2.0
        
        # Environment (obstacles for laser simulation)
        self.obstacles = [
            {'x': 3.0, 'y': 2.0, 'radius': 0.5},
            {'x': -2.0, 'y': 3.0, 'radius': 0.3},
            {'x': 4.0, 'y': -1.0, 'radius': 0.4},
            {'x': -3.0, 'y': -2.0, 'radius': 0.6},
            {'x': 0.0, 'y': 5.0, 'radius': 0.3},
        ]
        
        # Room boundaries
        self.walls = [
            {'x1': -5.0, 'y1': -5.0, 'x2': 5.0, 'y2': -5.0},   # Bottom
            {'x1': 5.0, 'y1': -5.0, 'x2': 5.0, 'y2': 5.0},     # Right
            {'x1': 5.0, 'y1': 5.0, 'x2': -5.0, 'y2': 5.0},     # Top
            {'x1': -5.0, 'y1': 5.0, 'x2': -5.0, 'y2': -5.0},   # Left
        ]
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Navigation goal subscriber
        self.goal_sub = self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        
        # Transform broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Current navigation goal
        self.current_goal = None
        self.goal_tolerance = 0.2
        
        # Timers
        self.timer = self.create_timer(0.05, self.update_robot_state)  # 20 Hz
        self.laser_timer = self.create_timer(0.1, self.publish_laser_scan)  # 10 Hz
        
        # Publish static transforms
        self.publish_static_transforms()
        
        self.get_logger().info('Enhanced Virtual Robot initialized with SLAM/Nav2 support')
    
    def publish_static_transforms(self):
        """Publish static transforms required by Nav2"""
        transforms = []
        
        # Base_link to laser frame
        laser_tf = TransformStamped()
        laser_tf.header.stamp = self.get_clock().now().to_msg()
        laser_tf.header.frame_id = 'base_link'
        laser_tf.child_frame_id = 'laser'
        laser_tf.transform.translation.x = 0.2   # Laser is 20cm in front
        laser_tf.transform.translation.y = 0.0
        laser_tf.transform.translation.z = 0.1   # 10cm high
        laser_tf.transform.rotation.w = 1.0
        transforms.append(laser_tf)
        
        # Base_link to base_footprint (used by Nav2)
        footprint_tf = TransformStamped()
        footprint_tf.header.stamp = self.get_clock().now().to_msg()
        footprint_tf.header.frame_id = 'base_link'
        footprint_tf.child_frame_id = 'base_footprint'
        footprint_tf.transform.translation.x = 0.0
        footprint_tf.transform.translation.y = 0.0
        footprint_tf.transform.translation.z = -0.1  # Footprint on ground
        footprint_tf.transform.rotation.w = 1.0
        transforms.append(footprint_tf)
        
        self.static_tf_broadcaster.sendTransform(transforms)
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.linear_vel = max(-self.max_linear_vel, 
                             min(self.max_linear_vel, msg.linear.x))
        self.angular_vel = max(-self.max_angular_vel, 
                              min(self.max_angular_vel, msg.angular.z))
        
        if abs(self.linear_vel) > 0.01 or abs(self.angular_vel) > 0.01:
            self.get_logger().info(f'Moving: linear={self.linear_vel:.2f}, angular={self.angular_vel:.2f}')
    
    def goal_callback(self, msg):
        """Handle navigation goals from web interface"""
        self.current_goal = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y
        }
        self.get_logger().info(f'New navigation goal: ({self.current_goal["x"]:.2f}, {self.current_goal["y"]:.2f})')
    
    def update_robot_state(self):
        """Update robot position and publish odometry"""
        dt = 0.05  # 20 Hz
        
        # Simple autonomous navigation if we have a goal and no manual control
        if (self.current_goal and 
            abs(self.linear_vel) < 0.01 and abs(self.angular_vel) < 0.01):
            self.navigate_to_goal()
        
        # Update robot pose using bicycle model
        self.x += self.linear_vel * math.cos(self.theta) * dt
        self.y += self.linear_vel * math.sin(self.theta) * dt
        self.theta += self.angular_vel * dt
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish odometry and transforms
        self.publish_odometry()
        self.publish_transforms()
    
    def navigate_to_goal(self):
        """Simple navigation controller"""
        if not self.current_goal:
            return
        
        # Calculate distance and angle to goal
        dx = self.current_goal['x'] - self.x
        dy = self.current_goal['y'] - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if goal is reached
        if distance < self.goal_tolerance:
            self.current_goal = None
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.get_logger().info('Navigation goal reached!')
            return
        
        # Calculate desired heading
        desired_theta = math.atan2(dy, dx)
        angle_diff = math.atan2(math.sin(desired_theta - self.theta), 
                               math.cos(desired_theta - self.theta))
        
        # Simple proportional controller
        if abs(angle_diff) > 0.1:  # Need to turn first
            self.angular_vel = 1.0 * math.copysign(1, angle_diff)
            self.linear_vel = 0.1  # Move slowly while turning
        else:  # Move toward goal
            self.angular_vel = 0.5 * angle_diff  # Fine corrections
            self.linear_vel = min(0.5, distance)  # Slow down as we approach
    
    def publish_odometry(self):
        """Publish robot odometry for Nav2"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from theta)
        q = euler_to_quaternion(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Velocity
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.angular.z = self.angular_vel
        
        # Covariance (important for Nav2)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y  
        odom.pose.covariance[35] = 0.01  # theta
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[35] = 0.01 # vtheta
        
        self.odom_pub.publish(odom)
    
    def publish_transforms(self):
        """Publish TF transforms"""
        # Odom -> base_link transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = euler_to_quaternion(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_laser_scan(self):
        """Publish detailed laser scan for SLAM and navigation"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        
        # Laser parameters (360 degree scan)
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0  # 1 degree resolution
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        # Calculate number of rays
        num_rays = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        ranges = []
        
        for i in range(num_rays):
            angle = scan.angle_min + i * scan.angle_increment
            world_angle = self.theta + angle
            
            # Cast ray and find closest obstacle
            min_distance = scan.range_max
            
            # Check walls
            for wall in self.walls:
                dist = self.ray_line_intersection(
                    self.x, self.y, world_angle,
                    wall['x1'], wall['y1'], wall['x2'], wall['y2']
                )
                if dist is not None and dist < min_distance:
                    min_distance = dist
            
            # Check circular obstacles
            for obstacle in self.obstacles:
                dist = self.ray_circle_intersection(
                    self.x, self.y, world_angle,
                    obstacle['x'], obstacle['y'], obstacle['radius']
                )
                if dist is not None and dist < min_distance:
                    min_distance = dist
            
            # Add some noise for realism
            if min_distance < scan.range_max:
                noise = np.random.normal(0, 0.01)  # 1cm standard deviation
                min_distance = max(scan.range_min, min_distance + noise)
            
            ranges.append(min_distance)
        
        scan.ranges = ranges
        self.laser_pub.publish(scan)
    
    def ray_line_intersection(self, rx, ry, angle, x1, y1, x2, y2):
        """Calculate intersection of ray with line segment"""
        # Ray direction
        dx = math.cos(angle)
        dy = math.sin(angle)
        
        # Line segment direction
        lx = x2 - x1
        ly = y2 - y1
        
        # Solve intersection
        denom = dx * ly - dy * lx
        if abs(denom) < 1e-10:
            return None  # Parallel lines
        
        t = ((x1 - rx) * ly - (y1 - ry) * lx) / denom
        u = ((x1 - rx) * dy - (y1 - ry) * dx) / denom
        
        if t > 0 and 0 <= u <= 1:
            return t
        else:
            return None
    
    def ray_circle_intersection(self, rx, ry, angle, cx, cy, radius):
        """Calculate intersection of ray with circle"""
        # Ray direction
        dx = math.cos(angle)
        dy = math.sin(angle)
        
        # Vector from ray origin to circle center
        fx = cx - rx
        fy = cy - ry
        
        # Quadratic equation coefficients
        a = dx*dx + dy*dy
        b = 2*(fx*dx + fy*dy)
        c = fx*fx + fy*fy - radius*radius
        
        discriminant = b*b - 4*a*c
        if discriminant < 0:
            return None
        
        t1 = (-b - math.sqrt(discriminant)) / (2*a)
        t2 = (-b + math.sqrt(discriminant)) / (2*a)
        
        # Return closest positive intersection
        if t1 > 0:
            return t1
        elif t2 > 0:
            return t2
        else:
            return None

def main(args=None):
    rclpy.init(args=args)
    robot = EnhancedVirtualRobot()
    
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()