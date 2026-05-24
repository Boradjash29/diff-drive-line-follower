#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import time

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.subscriber = self.create_subscription(
            Point,
            '/line_error',
            self.error_callback,
            10
        )
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # PID constants (tune these)
        self.kp = 0.015
        self.ki = 0.000
        self.kd = 0.005
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()
        
        self.get_logger().info('Controller Node Started')

    def error_callback(self, msg):
        # Check if line is lost
        if msg.z == 0.0:
            twist = Twist()
            twist.linear.x = 0.0
            # Rotate to find the line based on last known error
            if self.prev_error > 0:
                twist.angular.z = 0.5
            elif self.prev_error < 0:
                twist.angular.z = -0.5
            else:
                twist.angular.z = 0.0
            self.publisher.publish(twist)
            return

        error = msg.x
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        
        if dt <= 0.0:
            return
            
        # Proportional
        p_term = self.kp * error
        
        # Integral
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        # PID Output (correction)
        correction = p_term + i_term + d_term
        
        # Limit the correction (max angular velocity)
        max_angular_vel = 1.5
        if correction > max_angular_vel:
            correction = max_angular_vel
        elif correction < -max_angular_vel:
            correction = -max_angular_vel
            
        # Create Twist message
        twist = Twist()
        
        # Slow down on turns
        if abs(error) > 150:
            twist.linear.x = 0.05
        elif abs(error) > 75:
            twist.linear.x = 0.15
        else:
            twist.linear.x = 0.4
            
        twist.angular.z = correction
        
        # Publish velocity
        self.publisher.publish(twist)
        
        # Update state
        self.prev_error = error
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
