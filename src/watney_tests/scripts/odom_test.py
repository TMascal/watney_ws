#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MoveToXGoal(Node):
    def __init__(self, target_distance):
        super().__init__('move_to_x_goal')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.initial_x = None
        self.current_x = None
        self.target_distance = target_distance
        self.target_reached = False
        self.measured_error = None  # To store the error after the run

    def odom_callback(self, msg):
        if self.initial_x is None:
            self.initial_x = msg.pose.pose.position.x  # Store initial x position
            self.get_logger().info(f"Initial X: {self.initial_x}")

        self.current_x = msg.pose.pose.position.x

        if self.current_x - self.initial_x >= self.target_distance and not self.target_reached:
            self.target_reached = True
            self.measured_error = self.current_x - self.initial_x - self.target_distance
            self.get_logger().info(f"Target reached at X: {self.current_x}")
            self.stop_robot()

    def send_velocity(self, x_speed=0.2):
        if not self.target_reached:
            twist = Twist()
            twist.linear.x = x_speed
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Sending velocity command...")

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)  # Publish zero velocity
        self.get_logger().info("Stopping robot.")
        rclpy.shutdown()

def main():
    rclpy.init()

    while True:
        try:
            target_distance = float(input("Enter the target distance (in meters): "))
        except ValueError:
            print("Invalid input. Please enter a numeric value.")
            continue

        node = MoveToXGoal(target_distance)
        rate = node.create_rate(10)  # 10 Hz loop

        while rclpy.ok() and not node.target_reached:
            node.send_velocity()
            rclpy.spin_once(node, timeout_sec=0.1)  # Process callbacks

        # Print the measured error after the run
        if node.measured_error is not None:
            print(f"Measured error: {node.measured_error:.4f} meters")

        node.destroy_node()

        # Ask the user if they want to run the test again
        repeat = input("Do you want to run the test again? (yes/no): ").strip().lower()
        if repeat != 'yes':
            print("Exiting the program.")
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()