import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import logging
import serial
import threading
import json
import math

class SerialNode(Node):
    def __init__(self):
        super().__init__('h2l_node')
        self.declare_parameter('port', '/dev/serial0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('use_mag', False)
        self.declare_parameter('feedback_frequency', 25.0)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        feedback_freq = self.get_parameter('feedback_frequency').get_parameter_value().double_value
        self.use_mag = self.get_parameter('use_mag').get_parameter_value().bool_value
        self.accel_offsets = (0.0, 0.0, 0.0)
        self.gyro_offsets = (0.0, 0.0, 0.0)

        self.read_publisher = self.create_publisher(String, '/h2l_node/read', 10)
        self.chatter_publisher = self.create_publisher(String, '/h2l_node/chatter', 10)
        self.write_subscription = self.create_subscription(String, '/h2l_node/write', self.write_serial, 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.mag_publisher = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/h2l_node/wheel_odometry', 10)
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.handle_cmd_vel, 10)
        self.write_subscription  # prevent unused variable warning
        self.ser = serial.Serial(port, baudrate, dsrdtr=None)
        self.ser.setRTS(False)
        self.ser.setDTR(False)

        self.calibration_event = threading.Event()
        self.calibration_data = []  # Store calibration data temporarily
        self.serial_recv_thread = threading.Thread(target=self.read_serial)
        self.serial_recv_thread.daemon = True
        self.serial_recv_thread.start()

        self.last_time = self.get_clock().now()
        self.previous_odl_m = 0.0
        self.previous_odr_m = 0.0
        self.x_position = 0.0
        self.y_position = 0.0
        self.delta_d = 0.0
        self.theta = 0.0

        self.odometry_initialized = False


        self.last_cmd_vel_time = None

        self.set_feedback_rate(500.0)
        self.hl_calibrate_imu()
        self.set_feedback_rate(feedback_freq)
        self.get_logger().info(f"Feedback frequency set to {feedback_freq} Hz")
        self.velocity_timeout_timer = self.create_timer(0.1, self.check_velocity_timeout)
        self.get_logger().info(f"Command Executed. Defualt Values Initialized. There you are. Deploying!")

    def read_serial(self):
        while rclpy.ok():
            try:
                data = self.ser.readline().decode('utf-8')
                if data:
                    msg = String()
                    msg.data = data
                    self.read_publisher.publish(msg)

                    # If calibration is ongoing, collect data
                    if not self.calibration_event.is_set():
                        self.calibration_data.append(data)
                    else:
                        self.handle_json(data)
            except Exception as e:
                self.get_logger().warning(f"Error reading serial data: {e}. Discarding data and attempting next message.")
                self.ser.flush()  # Flush the serial input buffer

    def set_feedback_rate(self, rate_hz):
        json_data = {
            "T": 142,
            "cmd": int(1000 / rate_hz)  # Convert Hz to milliseconds
        }
        json_str = json.dumps(json_data)
        self.write_serial(String(data=json_str))

    def hl_calibrate_imu(self, num_samples=1000):
        self.get_logger().info("Starting IMU calibration... Keep sensor flat and steady! This is gonna take a hot second...")

        ax, ay, az, gx, gy, gz = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        # Wait for enough data to be collected
        while len(self.calibration_data) < num_samples:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Process the collected data
        for i in range(num_samples):
            try:
                data = self.calibration_data[i]
                json_data = json.loads(data)

                # Accumulate raw IMU data
                ax += float(json_data.get('ax', 0.0))
                ay += float(json_data.get('ay', 0.0))
                az += float(json_data.get('az', 0.0))
                gx += float(json_data.get('gx', 0.0))
                gy += float(json_data.get('gy', 0.0))
                gz += float(json_data.get('gz', 0.0))

            except Exception as e:
                self.get_logger().warning(f"Error during calibration data collection: {e}")
                continue

        # Compute average offsets
        accel_offset_x = ax / num_samples
        accel_offset_y = ay / num_samples
        accel_offset_z = (az / num_samples) - 8192  # Adjust Z to 1g (9.81 m/s^2)

        gyro_offset_x = gx / num_samples
        gyro_offset_y = gy / num_samples
        gyro_offset_z = gz / num_samples

        self.get_logger().info("IMU Calibration complete!")
        self.get_logger().info(f"Accel Offsets: x={accel_offset_x}, y={accel_offset_y}, z={accel_offset_z}")
        self.get_logger().info(f"Gyro Offsets: x={gyro_offset_x}, y={gyro_offset_y}, z={gyro_offset_z}")

        # Store offsets for later use
        self.accel_offsets = (accel_offset_x, accel_offset_y, accel_offset_z)
        self.gyro_offsets = (gyro_offset_x, gyro_offset_y, gyro_offset_z)

        # Signal that calibration is complete
        self.calibration_event.set()

    def handle_json(self, data):
        try:
            json_data = json.loads(data)
            if 'T' in json_data:
                t_value = json_data['T']
                handlers = {
                    1001: lambda data: self.handle_1001(data, self.use_mag),
                }
                handler = handlers.get(t_value, self.handle_default(json_data))
                handler(json_data)
            else:
                self.get_logger().debug("Non-JSON or garbled message received")
                chatter_msg = String()
                chatter_msg.data = data
                self.chatter_publisher.publish(chatter_msg)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e}")
            self.get_logger().debug("Non-JSON or garbled message received")
            chatter_msg = String()
            chatter_msg.data = data
            self.chatter_publisher.publish(chatter_msg)

    def handle_cmd_vel(self, msg):
        self.last_cmd_vel_time = self.get_clock().now()
        json_data = {
            "T": 13,
            "X": msg.linear.x,
            "Z": msg.angular.z
        }
        json_str = json.dumps(json_data)
        self.write_serial(String(data=json_str))

    def check_velocity_timeout(self):
        """Check if no velocity command has been received for 3 seconds."""
        if self.last_cmd_vel_time is None:
            return  # Do nothing if no velocity commands have been received

        current_time = self.get_clock().now()
        time_since_last_cmd = (current_time - self.last_cmd_vel_time).nanoseconds / 1e9  # Convert to seconds

        if time_since_last_cmd > 3.0:
            # Send zero velocity if timeout occurs
            self.get_logger().info("Velocity timeout detected. Sending zero velocity.")
            self.handle_cmd_vel(Twist())  # Send zero velocity
            self.last_cmd_vel_time = None

    def handle_default(self, json_data):
        self.get_logger().debug(f"Received message with value: {json_data['T']}")


    def handle_1001(self, json_data, use_mag):
        current_time = self.get_clock().now()
        self.last_time = current_time

        # IMU Data

        #Sensitivity Scale Factors (see ICM20948 data sheet)
        accel_ssf = 8192
        gyro_ssf = 32.8
        magn_ssf = 0.15

        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = "imu_link"
        if use_mag is False:
            imu_msg.orientation_covariance = [
                0.0028, -0.0001, 0.0033,
                -0.0001, 0.0001, 0.0010,
                0.0033, 0.0010, 8.3553
            ]  # Calculated Covariances in Matlab w/o Magnetometer
        else:
            imu_msg.orientation_covariance = [
                0.0016, -0.0000, 0.0008,
                -0.0000, 0.0000, -0.0009,
                0.0008, -0.0009, 7.2158
            ]  # Calculated Covariances in Matlab w/ Magnetometer

        imu_msg.angular_velocity.x = ((float(json_data.get('gx', 0.0)) - self.gyro_offsets[0]) / gyro_ssf) * (math.pi / 180.0)
        imu_msg.angular_velocity.y = ((float(json_data.get('gy', 0.0)) - self.gyro_offsets[1]) / gyro_ssf) * (math.pi / 180.0)
        imu_msg.angular_velocity.z = ((float(json_data.get('gz', 0.0)) - self.gyro_offsets[2]) / gyro_ssf) * (math.pi / 180.0)
        imu_msg.angular_velocity_covariance = [
            0.5669e-06, 0.0285e-06, -0.0037e-06,
            0.0285e-06, 0.5614e-06, 0.0141e-06,
            -0.0037e-06, 0.0141e-06, 0.4967e-06
        ] # Calculated Covariances in Matlab
        
        imu_msg.linear_acceleration.x = (float(json_data.get('ax', 0.0)) - self.accel_offsets[0]) / accel_ssf * 9.81
        imu_msg.linear_acceleration.y = (float(json_data.get('ay', 0.0)) - self.accel_offsets[1]) / accel_ssf * 9.81
        imu_msg.linear_acceleration.z = (float(json_data.get('az', 0.0)) - self.accel_offsets[2]) / accel_ssf * 9.81
        imu_msg.linear_acceleration_covariance = [
            0.0015, 0.0000, -0.0000,
            0.0000, 0.0014, 0.0000,
            -0.0000, 0.0000, 0.0018
        ] # Calculated Covariances in Matlab

        self.imu_publisher.publish(imu_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp = current_time.to_msg()
        mag_msg.header.frame_id = "imu_link"
        mag_msg.magnetic_field.x = float(json_data.get('mx', 0.0)) * magn_ssf
        mag_msg.magnetic_field.y = float(json_data.get('my', 0.0)) * magn_ssf
        mag_msg.magnetic_field.z = float(json_data.get('mz', 0.0)) * magn_ssf
        mag_msg.magnetic_field_covariance = [
            0.9115, -0.1343, -0.2316,
           -0.1343,  0.6865,  0.0859,
           -0.2316,  0.0859,  0.7230
        ] # Calculated Covariances in Matlab

        self.mag_publisher.publish(mag_msg)

        # Odometry and Position

        #Physical Constants
        width = 0.172
        wheel_diameter = 0.08

        lVel = float(json_data.get('L', 0.0))
        rVel = float(json_data.get('R', 0.0))
        odl_cm = float(json_data.get('odl', 0.0))
        odr_cm = float(json_data.get('odr', 0.0))

        odl_m = odl_cm / 100.0
        odr_m = odr_cm / 100.0

        # Joint States
        left_wheel_position = 2.0 * odl_m / wheel_diameter
        right_wheel_position = 2.0 * odr_m / wheel_diameter
        left_wheel_velocity = 2.0 * lVel / wheel_diameter
        right_wheel_velocity = 2.0 * rVel / wheel_diameter

        # Odom
        linear_velocity_x = (rVel + lVel) / 2.0
        angular_velocity_z = (rVel - lVel) / width

                # Use a flag to ensure initialization happens only once
        if not self.odometry_initialized:
            self.previous_odl_m = odl_m
            self.previous_odr_m = odr_m
            delta_d = 0.0  # No movement yet
            delta_theta = 0.0
            self.odometry_initialized = True  # Mark that initialization is done
        else:
            # Compute differences from previous readings
            delta_d = ((odl_m - self.previous_odl_m) + (odr_m - self.previous_odr_m)) / 2.0
            delta_theta = ((odr_m - self.previous_odr_m) - (odl_m - self.previous_odl_m)) / width

            # Update previous readings
            self.previous_odl_m = odl_m
            self.previous_odr_m = odr_m

        self.delta_d = delta_d

        scale_factor = 3.5
        self.x_position += self.delta_d * scale_factor * math.cos(self.theta + delta_theta / 2.0)
        self.y_position += self.delta_d * scale_factor * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # ----- Improved Covariance Estimates for Odometry -----
        # Pose covariance: Order is [x, y, z, roll, pitch, yaw]
        # For a flat ground robot:
        #  - Low variance on x and y (good encoder accuracy)
        #  - Extremely high variance on z, roll, and pitch (not observed)
        #  - Moderate variance on yaw (subject to drift)
        odom_pose_cov = [
            0.02,    0.0,     0.0,     0.0,     0.0,     0.0,   # x
            0.0,     0.02,    0.0,     0.0,     0.0,     0.0,   # y
            0.0,     0.0,  99999.0,    0.0,     0.0,     0.0,   # z
            0.0,     0.0,     0.0,  99999.0,    0.0,     0.0,   # roll
            0.0,     0.0,     0.0,     0.0,  99999.0,    0.0,   # pitch
            0.0,     0.0,     0.0,     0.0,     0.0,     0.2    # yaw
        ]
        
        # Twist covariance: Order is [linear x, y, z, angular x, y, z]
        # We trust forward linear and yaw rate measurements more,
        # and we assign high uncertainty to unused dimensions.
        odom_twist_cov = [
            0.01,    0.0,     0.0,     0.0,     0.0,     0.0,   # linear x
            0.0,     0.01,    0.0,     0.0,     0.0,     0.0,   # linear y
            0.0,     0.0,  99999.0,    0.0,     0.0,     0.0,   # linear z
            0.0,     0.0,     0.0,  99999.0,    0.0,     0.0,   # angular x
            0.0,     0.0,     0.0,     0.0,  99999.0,    0.0,   # angular y
            0.0,     0.0,     0.0,     0.0,     0.0,     0.2    # angular z
        ]

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = self.x_position
        odom_msg.pose.pose.position.y = self.y_position
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)
        odom_msg.twist.twist.linear.x = linear_velocity_x
        odom_msg.twist.twist.angular.z = angular_velocity_z
        odom_msg.pose.covariance = odom_pose_cov
        odom_msg.twist.covariance = odom_twist_cov

        self.odom_publisher.publish(odom_msg)

        #add joint publisher
        left_upper_joint_msg = JointState()
        left_upper_joint_msg.header.stamp = current_time.to_msg()
        left_upper_joint_msg.name = ["left_up_wheel_link_joint"]
        left_upper_joint_msg.position = [left_wheel_position]
        left_upper_joint_msg.velocity = [left_wheel_velocity]

        right_upper_joint_msg = JointState()
        right_upper_joint_msg.header.stamp = current_time.to_msg()
        right_upper_joint_msg.name = ["right_up_wheel_link_joint"]
        right_upper_joint_msg.position = [right_wheel_position]
        right_upper_joint_msg.velocity = [right_wheel_velocity]

        left_down_joint_msg = JointState()
        left_down_joint_msg.header.stamp = current_time.to_msg()
        left_down_joint_msg.name = ["left_down_wheel_link_joint"]
        left_down_joint_msg.position = [left_wheel_position]
        left_down_joint_msg.velocity = [left_wheel_velocity]

        right_down_joint_msg = JointState()
        right_down_joint_msg.header.stamp = current_time.to_msg()
        right_down_joint_msg.name = ["right_down_wheel_link_joint"]
        right_down_joint_msg.position = [right_wheel_position]
        right_down_joint_msg.velocity = [right_wheel_velocity]

        self.joint_publisher.publish(left_upper_joint_msg)
        self.joint_publisher.publish(right_upper_joint_msg)
        self.joint_publisher.publish(left_down_joint_msg)
        self.joint_publisher.publish(right_down_joint_msg)

    def write_serial(self, msg):
        self.ser.write(msg.data.encode() + b'\n')

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()