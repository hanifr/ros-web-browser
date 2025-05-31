#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import time
import math

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        
        # Monitoring variables
        self.last_cmd_vel = None
        self.last_odom = None
        self.last_scan = None
        self.last_status = None
        
        self.cmd_vel_count = 0
        self.odom_count = 0
        self.scan_count = 0
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/robot_status', self.status_callback, 10)
        
        # Monitor timer
        self.monitor_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('System monitor started')
    
    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg
        self.cmd_vel_count += 1
    
    def odom_callback(self, msg):
        self.last_odom = msg
        self.odom_count += 1
    
    def scan_callback(self, msg):
        self.last_scan = msg
        self.scan_count += 1
    
    def status_callback(self, msg):
        self.last_status = msg.data
    
    def print_status(self):
        print("\n" + "="*60)
        print("🤖 ROBOT SYSTEM MONITOR")
        print("="*60)
        
        # Message counts
        print(f"📊 Message Rates (2s window):")
        print(f"   cmd_vel: {self.cmd_vel_count/2:.1f} Hz")
        print(f"   odom:    {self.odom_count/2:.1f} Hz") 
        print(f"   scan:    {self.scan_count/2:.1f} Hz")
        
        # Reset counters
        self.cmd_vel_count = 0
        self.odom_count = 0
        self.scan_count = 0
        
        # Current velocities
        if self.last_cmd_vel:
            print(f"🎮 Command Velocity:")
            print(f"   Linear:  {self.last_cmd_vel.linear.x:.3f} m/s")
            print(f"   Angular: {self.last_cmd_vel.angular.z:.3f} rad/s")
        
        # Current pose
        if self.last_odom:
            pos = self.last_odom.pose.pose.position
            orientation = self.last_odom.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
            )
            
            print(f"📍 Robot Pose:")
            print(f"   Position: ({pos.x:.3f}, {pos.y:.3f})")
            print(f"   Heading:  {math.degrees(yaw):.1f}°")
            
            twist = self.last_odom.twist.twist
            actual_linear = math.sqrt(twist.linear.x**2 + twist.linear.y**2)
            print(f"   Actual Linear:  {actual_linear:.3f} m/s")
            print(f"   Actual Angular: {twist.angular.z:.3f} rad/s")
        
        # Laser scan info
        if self.last_scan:
            ranges = [r for r in self.last_scan.ranges if r > 0 and r < self.last_scan.range_max]
            if ranges:
                min_range = min(ranges)
                print(f"🔴 Laser Scan:")
                print(f"   Points:    {len(ranges)}/{len(self.last_scan.ranges)}")
                print(f"   Min Range: {min_range:.3f} m")
                print(f"   Range:     {self.last_scan.range_min:.1f}-{self.last_scan.range_max:.1f} m")
        
        # Robot status
        if self.last_status:
            print(f"🤖 Robot Status: {self.last_status}")
        
        # System health
        current_time = self.get_clock().now()
        print(f"⏰ System Time: {current_time.to_msg().sec}s")

def main():
    rclpy.init()
    monitor = SystemMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nMonitor stopped")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()