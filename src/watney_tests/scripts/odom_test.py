#!/usr/bin/env python3

import time
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
        self.max_vel = 0.1

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

    def send_velocity(self):
        if self.initial_x is None:
            self.get_logger().info("Waiting for odom message...")
            return
        if not self.target_reached:
            twist = Twist()
            twist.linear.x = self.max_vel
            self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)  # Publish zero velocity
        self.get_logger().info("Stopping robot.")

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
        safety_factor = 1.5
        expected_time = target_distance / node.max_vel * safety_factor
        start_time = time.time()

        while rclpy.ok() and not node.target_reached:
            node.send_velocity()
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = time.time() - start_time
            if node.initial_x is not None and node.current_x is not None:
                elapsed_distance = node.current_x - node.initial_x
                node.get_logger().info(f"Elapsed Distance: {elapsed_distance:.4f} meters")
            if elapsed > expected_time:
                node.get_logger().warning("Test failed: runtime exceeded safety limit.")
                node.stop_robot()
                break

        # Print the measured error after the run if target reached
        if node.target_reached:
            extra_samples = []
            sampling_duration = 1.0  # seconds
            sample_interval = 0.1    # seconds (10Hz)
            start_extra = time.time()
            while time.time() - start_extra < sampling_duration:
                rclpy.spin_once(node, timeout_sec=sample_interval)
                if node.current_x is not None:
                    extra_samples.append(node.current_x)
                time.sleep(sample_interval)
            
            if extra_samples:
                final_x = extra_samples[-1]
                overall_error = final_x - node.initial_x - node.target_distance
            else:
                overall_error = node.measured_error
            
            print("Test Successful!")
            print(f"Measured error after extra sampling: {overall_error:.4f} meters")
        elif not node.target_reached:
            pass

        node.destroy_node()

        # Ask the user if they want to run the test again
        repeat = input("Do you want to run the test again? (yes/no): ").strip().lower()
        if repeat != 'yes':
            print("Exiting the program.")
            break

    rclpy.shutdown()

if __name__ == '__main__':
    main()