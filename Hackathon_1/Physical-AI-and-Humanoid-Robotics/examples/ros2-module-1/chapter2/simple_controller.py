#!/usr/bin/env python3
"""
Simple Controller Example for Python Agents Controlling ROS Systems

This example demonstrates how to create a Python-based ROS 2 controller
that subscribes to sensor data and publishes commands to control a simulated robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist


class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        # Create subscription to sensor data (e.g., obstacle distance)
        self.subscription = self.create_subscription(
            Float64,
            'sensor_distance',
            self.sensor_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create publisher for robot commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a timer for periodic status updates
        self.timer = self.create_timer(1.0, self.status_timer_callback)

        # Controller state
        self.last_distance = 0.0
        self.robot_status = "IDLE"

        self.get_logger().info('Simple controller node initialized')

    def sensor_callback(self, msg):
        """Process incoming sensor data and generate appropriate commands."""
        distance = msg.data
        self.last_distance = distance

        # Simple control logic: stop if obstacle is too close, move forward otherwise
        cmd_msg = Twist()

        if distance < 0.5:  # Obstacle too close (less than 0.5 meters)
            cmd_msg.linear.x = 0.0  # Stop
            cmd_msg.angular.z = 0.0
            self.robot_status = "STOPPED - OBSTACLE DETECTED"
            self.get_logger().warn('Obstacle detected! Stopping robot.')
        else:
            cmd_msg.linear.x = 0.2  # Move forward at 0.2 m/s
            cmd_msg.angular.z = 0.0
            self.robot_status = "MOVING FORWARD"

        # Publish the command
        self.publisher.publish(cmd_msg)
        self.get_logger().info(f'Published command: linear.x={cmd_msg.linear.x}, status: {self.robot_status}')

    def status_timer_callback(self):
        """Periodically publish robot status."""
        status_msg = String()
        status_msg.data = f'Robot Status: {self.robot_status}, Last Distance: {self.last_distance:.2f}m'

        # You could publish this to a status topic if needed
        # For now, just log it
        self.get_logger().info(status_msg.data)


def main(args=None):
    rclpy.init(args=args)
    controller = SimpleController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Controller stopped by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()