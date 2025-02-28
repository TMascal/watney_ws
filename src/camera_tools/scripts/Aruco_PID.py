import cv2
import numpy as np
import time
import rclpy
from rclpy.node import Node
from simple_pid import PID
from geometry_msgs.msg import Twist

class ArUcoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.process_frame)  # 10 Hz
        self.cap = cv2.VideoCapture(0)

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.parameters = cv2.aruco.DetectorParameters()
        self.K = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])  
        self.dist_coeffs = np.zeros((5, 1))
        self.prev_transformation_matrix = None
        self.prev_time = time.time()

        self.desired_position = np.array([0, 0, 0.5])
        self.max_velocity = 2

        # PID Controllers
        self.pid_x = PID(1, 0, 0, setpoint=0)
        self.pid_y = PID(1, 0, 0, setpoint=0)
        self.pid_z = PID(1, 0, 0, setpoint=0)

    def get_transformation_matrix(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        T = np.hstack((R, tvec.reshape(3, 1)))
        return np.vstack((T, [0, 0, 0, 1]))

    def compute_velocity(self, prev_matrix, curr_matrix, dt):
        if prev_matrix is None:
            return np.zeros(6)
        translation_diff = curr_matrix[:3, 3] - prev_matrix[:3, 3]
        rotation_diff = curr_matrix[:3, :3] @ np.linalg.inv(prev_matrix[:3, :3])
        rvec_diff, _ = cv2.Rodrigues(rotation_diff)
        velocity = np.hstack((translation_diff, rvec_diff.flatten())) / dt
        return velocity

    def process_frame(self):
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

                curr_transformation_matrix = self.get_transformation_matrix(rvec[0], tvec[0])
                curr_time = time.time()
                dt = curr_time - self.prev_time
                velocity = self.compute_velocity(self.prev_transformation_matrix, curr_transformation_matrix, dt)

                position_error = curr_transformation_matrix[:3, 3] - self.desired_position
                control_signal_x = self.pid_x(position_error[0])
                control_signal_y = self.pid_y(position_error[1])
                control_signal_z = self.pid_z(position_error[2])

                control_signal_x = np.clip(control_signal_x, -self.max_velocity, self.max_velocity)
                control_signal_y = np.clip(control_signal_y, -self.max_velocity, self.max_velocity)
                control_signal_z = np.clip(control_signal_z, -self.max_velocity, self.max_velocity)

                angular_z = np.arctan2(control_signal_y, control_signal_x)

                # Create and publish Twist message
                twist_msg = Twist()
                twist_msg.linear.x = control_signal_x
                twist_msg.angular.z = angular_z
                self.publisher.publish(twist_msg)

                self.get_logger().info(f"Published: X={control_signal_x:.2f} m/s, Z={angular_z:.2f} rad/s")

                self.prev_transformation_matrix = curr_transformation_matrix
                self.prev_time = curr_time

        cv2.imshow('ArUco Tracker', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main():
    rclpy.init()
    tracker = ArUcoTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
