#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
import time
import math

class RobotTester(Node):
    def __init__(self):
        super().__init__('robot_tester')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(Point, '/move_base_simple/goal', 10)
        self.mode_pub = self.create_publisher(String, '/behavior_mode', 10)
        
        self.get_logger().info('Robot tester node started')
        self.get_logger().info('Available test commands:')
        self.get_logger().info('  square() - Move in square pattern')
        self.get_logger().info('  circle() - Move in circle pattern') 
        self.get_logger().info('  test_modes() - Test autonomous modes')
        self.get_logger().info('  emergency_stop() - Stop robot immediately')
    
    def square_pattern(self, side_length=2.0, speed=0.3):
        """Move robot in a square pattern"""
        self.get_logger().info(f'Starting square pattern (side: {side_length}m, speed: {speed}m/s)')
        
        for i in range(4):
            # Move forward
            self.get_logger().info(f'Side {i+1}: Moving forward')
            self.move_forward(speed, side_length / speed)
            
            # Turn 90 degrees
            self.get_logger().info(f'Side {i+1}: Turning')
            self.turn_angle(math.pi/2, 0.5)
            
            time.sleep(0.5)  # Brief pause
        
        self.stop()
        self.get_logger().info('Square pattern completed')
    
    def circle_pattern(self, radius=1.0, angular_speed=0.5):
        """Move robot in a circle pattern"""
        linear_speed = angular_speed * radius
        duration = 2 * math.pi / angular_speed  # Full circle
        
        self.get_logger().info(f'Starting circle pattern (radius: {radius}m)')
        
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)
        
        self.stop()
        self.get_logger().info('Circle pattern completed')
    
    def test_autonomous_modes(self):
        """Test different autonomous behavior modes"""
        modes = ['obstacle_avoidance', 'explore', 'follow_wall', 'idle']
        
        for mode in modes:
            self.get_logger().info(f'Testing mode: {mode}')
            mode_msg = String()
            mode_msg.data = mode
            self.mode_pub.publish(mode_msg)
            
            # Run for 10 seconds
            time.sleep(10)
        
        # Return to idle
        mode_msg = String()
        mode_msg.data = 'idle'
        self.mode_pub.publish(mode_msg)
        self.get_logger().info('Autonomous mode testing completed')
    
    def move_forward(self, speed, duration):
        """Move forward at given speed for given duration"""
        cmd = Twist()
        cmd.linear.x = speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)
    
    def turn_angle(self, angle, angular_speed):
        """Turn by given angle at given angular speed"""
        duration = abs(angle) / angular_speed
        
        cmd = Twist()
        cmd.angular.z = angular_speed if angle > 0 else -angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)
    
    def send_goal(self, x, y):
        """Send goal point to robot"""
        goal = Point()
        goal.x = x
        goal.y = y
        goal.z = 0.0
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Goal sent: ({x}, {y})')
    
    def stop(self):
        """Stop robot immediately"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Robot stopped')

def main():
    rclpy.init()
    tester = RobotTester()
    
    # Interactive testing
    print("\n🤖 Robot Testing Interface")
    print("Available commands:")
    print("  1 - Square pattern")
    print("  2 - Circle pattern")
    print("  3 - Test autonomous modes")
    print("  4 - Send goal (interactive)")
    print("  s - Emergency stop")
    print("  q - Quit")
    
    try:
        while True:
            command = input("\nEnter command: ").strip().lower()
            
            if command == '1':
                tester.square_pattern()
            elif command == '2':
                tester.circle_pattern()
            elif command == '3':
                tester.test_autonomous_modes()
            elif command == '4':
                try:
                    x = float(input("Goal X: "))
                    y = float(input("Goal Y: "))
                    tester.send_goal(x, y)
                except ValueError:
                    print("Invalid coordinates")
            elif command == 's':
                tester.stop()
            elif command == 'q':
                break
            else:
                print("Unknown command")
    
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()