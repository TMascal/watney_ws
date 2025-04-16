import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger  # Import the Trigger service

class ArUcoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image,
            '/side_cam/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz timer

        # ArUco dictionary and parameters
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.parameters = cv2.aruco.DetectorParameters()
        self.K = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])
        self.dist_coeffs = np.zeros((5, 1))

        # Desired
        self.desired_yaw = 0.0

        # X-axis PID control
        self.Kp_lin, self.Ki_lin, self.Kd_lin = 1.0, 0.0, 0.0
        self.prev_error = 0.0
        self.integral = 0.0

        # Yaw PID control
        self.Kp_yaw, self.Ki_yaw, self.Kd_yaw = 1.0, 0.0, 0.0
        self.prev_yaw_error = 0.0
        self.yaw_integral = 0.0

        self.current_frame = None  # Store the latest frame
        self.active = False  # Track whether the node is active

        # Create a service to start/stop the node
        self.service = self.create_service(Trigger, 'start_aruco_tracking', self.handle_service)

    def handle_service(self, request, response):
        """Service callback to start or stop the node."""
        self.active = not self.active
        if self.active:
            self.get_logger().info("ArUco tracking started.")
            response.message = "ArUco tracking started."
        else:
            self.get_logger().info("ArUco tracking stopped.")
            response.message = "ArUco tracking stopped."
        response.success = True
        return response

    def image_callback(self, msg):
        """Callback to process incoming images from /side_cam/image_raw."""
        if not self.active:
            return  # Ignore images if the node is not active
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def timer_callback(self):
        if not self.active or self.current_frame is None:
            return  # Skip processing if the node is not active or no frame is available

        frame = self.current_frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        if ids is not None:
            frame_height, frame_width = frame.shape[:2]
            frame_center_x = frame_width / 2  # Center of the frame in the X-axis

            for i in range(len(ids)):
                marker_corners = corners[i]
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners, 0.05, self.K, self.dist_coeffs)
                cv2.drawFrameAxes(frame, self.K, self.dist_coeffs, rvec[0], tvec[0], 0.03)

                # X-axis alignment PID control
                marker_center_x = np.mean(marker_corners[0][:, 0])  # Average X-coordinate of marker corners
                x_error = frame_center_x - marker_center_x
                self.integral += x_error
                derivative = x_error - self.prev_error
                x_control_output = self.Kp_lin * x_error + self.Ki_lin * self.integral + self.Kd_lin * derivative
                x_control_output = 0.1 * x_control_output  # Scale the control output
                x_control_output = max(min(x_control_output, 1.0), -1.0)  # Cap the output between -1.0 and 1.0
                self.prev_error = x_error

                # Yaw PID control
                rotation_matrix, _ = cv2.Rodrigues(rvec[0])
                yaw = np.arctan2(rotation_matrix[2, 0], rotation_matrix[0, 0])
                yaw_error = self.desired_yaw - yaw
                self.yaw_integral += yaw_error
                yaw_derivative = yaw_error - self.prev_yaw_error
                yaw_control_output = self.Kp_yaw * yaw_error + self.Ki_yaw * self.yaw_integral + self.Kd_yaw * yaw_derivative
                yaw_control_output = 0.5 * yaw_control_output  # Scale the yaw control output
                self.prev_yaw_error = yaw_error

                # Publish Twist message
                twist_msg = Twist()
                twist_msg.linear.x = float(x_control_output)  # X-axis alignment control
                twist_msg.linear.y = 0.0  # No Y-axis motion
                twist_msg.angular.z = yaw_control_output  # Angular velocity for yaw control
                self.publisher.publish(twist_msg)
                self.get_logger().info(f"X Control: {x_control_output:.2f}, Yaw Control: {yaw_control_output:.2f}")

                # Stop the node if a certain condition is met (e.g., marker is centered)
                if abs(x_error) < 5 and abs(yaw_error) < 0.1:  # Example condition
                    self.get_logger().info("Marker aligned. Stopping ArUco tracking.")
                    self.active = False

def main(args=None):
    rclpy.init(args=args)
    tracker = ArUcoTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()