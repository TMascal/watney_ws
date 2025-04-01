import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ArUcoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz timer

        # ArUco dictionary and parameters
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.parameters = cv2.aruco.DetectorParameters()
        self.K = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])
        self.dist_coeffs = np.zeros((5, 1))

        # Desired
        self.desired_distance = 0.5
        self.desired_yaw = 0.0

        # Distance
        self.Kp_lin, self.Ki_lin, self.Kd_lin = 1.0, 0.0, 0.0
        self.prev_error = 0.0
        self.integral = 0.0

        # Yaw
        self.Kp_yaw, self.Ki_yaw, self.Kd_yaw = 1.0, 0.0, 0.0
        self.prev_yaw_error = 0.0
        self.yaw_integral = 0.0

        self.cap = cv2.VideoCapture(0)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        if ids is not None:
            for i in range(len(ids)):
                marker_corners = corners[i]
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners, 0.05, self.K, self.dist_coeffs)
                cv2.drawFrameAxes(frame, self.K, self.dist_coeffs, rvec[0], tvec[0], 0.03)

                # Distance PID control
                current_distance = np.linalg.norm(tvec[0][0])
                error = current_distance - self.desired_distance
                self.integral += error
                derivative = error - self.prev_error
                control_output = self.Kp_lin * error + self.Ki_lin * self.integral + self.Kd_lin * derivative
                control_output = 0.1 * control_output
                self.prev_error = error

                # Yaw PID control
                rotation_matrix, _ = cv2.Rodrigues(rvec[0])
                yaw = np.arctan2(rotation_matrix[2, 0], rotation_matrix[0, 0])
                yaw_error = self.desired_yaw - yaw
                self.yaw_integral += yaw_error
                yaw_derivative = yaw_error - self.prev_yaw_error
                yaw_control_output = self.Kp_yaw * yaw_error + self.Ki_yaw * self.yaw_integral + self.Kd_yaw * yaw_derivative
                yaw_control_output = 0.5 * yaw_control_output
                self.prev_yaw_error = yaw_error

                # Publish Twist message
                twist_msg = Twist()
                twist_msg.linear.x = control_output  # Linear velocity
                twist_msg.angular.z = yaw_control_output  # Angular velocity
                self.publisher.publish(twist_msg)
                self.get_logger().info(f"Distance Control: {control_output:.2f}, Yaw Control: {yaw_control_output:.2f}")

                # Display information
                cX, cY = int(marker_corners[0][0][0]), int(marker_corners[0][0][1])
                cv2.putText(frame, f"ID: {ids[i][0]} Dist: {current_distance:.2f}m", (cX, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"Dist Control: {control_output:.2f}", (cX, cY - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.putText(frame, f"Yaw Control: {yaw_control_output:.2f}", (cX, cY - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.imshow('ArUco Tracker', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    tracker = ArUcoTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()