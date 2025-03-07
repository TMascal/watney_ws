import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
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

        self.read_publisher = self.create_publisher(String, '/h2l_node/read', 10)
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
        self.serial_recv_thread = threading.Thread(target=self.read_serial)
        self.serial_recv_thread.daemon = True
        self.serial_recv_thread.start()

        self.last_time = self.get_clock().now()
        self.previous_odl = 0.0
        self.previous_odr = 0.0
        self.x_position = 0.0
        self.y_position = 0.0
        self.theta = 0.0

        self.set_feedback_rate(feedback_freq)
        self.get_logger().info(f"Feedback frequency set to {feedback_freq} Hz")
        self.imu_calibration()
        self.get_logger().info(f"IMU calibrating, please wait.")
        self.get_logger().info(f"Command Executed. Defualt Values Initialized. There you are. Deploying!")

    def read_serial(self):
        while rclpy.ok():
            data = self.ser.readline().decode('utf-8')
            if data:
                msg = String()
                msg.data = data
                self.read_publisher.publish(msg)
                self.handle_json(data)

    def set_feedback_rate(self, rate_hz):
        json_data = {
            "T": 142,
            "cmd": int(1000 / rate_hz)  # Convert Hz to milliseconds
        }
        json_str = json.dumps(json_data)
        self.write_serial(String(data=json_str))

    def imu_calibration(self):
        json_data_127 = {
            "T": 127
        }
        json_str_127 = json.dumps(json_data_127)
        self.write_serial(String(data=json_str_127))

        json_data_128 = {
            "T": 128
        }
        json_str_128 = json.dumps(json_data_128)
        self.write_serial(String(data=json_str_128))

    def handle_json(self, data):
        try:
            json_data = json.loads(data)
            if 'T' in json_data:
                t_value = json_data['T']
                handlers = {
                    1001: lambda data: self.handle_1001(data, self.use_mag),
                }
                handler = handlers.get(t_value, self.handle_default)
                handler(json_data)
            else:
                self.get_logger().warning("No 'T' value found in JSON")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e}")

    def handle_cmd_vel(self, msg):
        json_data = {
            "T": 13,
            "X": msg.linear.x,
            "Z": msg.angular.z
        }
        json_str = json.dumps(json_data)
        self.write_serial(String(data=json_str))

    def handle_1001(self, json_data, use_mag):
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time

        #Physical Constants
        width = 0.172
        wheel_radius = .08
        #Sensitivity Scale Factors (see ICM20948 data sheet)
        accel_ssf = 8192
        gyro_ssf = 65.5
        magn_ssf = 0.15


        lVel = float(json_data.get('L', 0.0))
        rVel = float(json_data.get('R', 0.0))
        odl = float(json_data.get('odl', 0.0))/100 # Convert to meters
        odr = float(json_data.get('odr', 0.0))/100 # Convert to meters

        delta_odl = odl - self.previous_odl
        delta_odr = odr - self.previous_odr
        left_wheel_position = delta_odl / wheel_radius
        right_wheel_position = delta_odr / wheel_radius
        self.previous_odl = odl
        self.previous_odr = odr

        linear_velocity_x = (rVel + lVel) / 2
        angular_velocity_z = (rVel - lVel) / width

        delta_theta = angular_velocity_z * delta_time
        self.theta += delta_theta

        delta_x = linear_velocity_x * delta_time * math.cos(self.theta + delta_theta / 2)
        delta_y = linear_velocity_x * delta_time * math.sin(self.theta + delta_theta / 2)

        self.x_position += delta_x
        self.y_position += delta_y

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

        imu_msg.angular_velocity.x = float(json_data.get('gx', 0.0))/gyro_ssf * (math.pi / 180.0)
        imu_msg.angular_velocity.y = float(json_data.get('gy', 0.0))/gyro_ssf * (math.pi / 180.0)
        imu_msg.angular_velocity.z = float(json_data.get('gz', 0.0))/gyro_ssf * (math.pi / 180.0)
        imu_msg.angular_velocity_covariance = [
            0.5669e-06, 0.0285e-06, -0.0037e-06,
            0.0285e-06, 0.5614e-06, 0.0141e-06,
            -0.0037e-06, 0.0141e-06, 0.4967e-06
        ] # Calculated Covariances in Matlab
        
        imu_msg.linear_acceleration.x = float(json_data.get('ax', 0.0)) / accel_ssf * 9.81
        imu_msg.linear_acceleration.y = float(json_data.get('ay', 0.0)) / accel_ssf * 9.81
        imu_msg.linear_acceleration.z = float(json_data.get('az', 0.0)) / accel_ssf * 9.81
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

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = self.x_position
        odom_msg.pose.pose.position.y = self.y_position
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)
        odom_msg.twist.twist.linear.x = linear_velocity_x
        odom_msg.twist.twist.angular.z = angular_velocity_z
        odom_msg.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ] # Filler values

        self.odom_publisher.publish(odom_msg)

        #add joint publisher
        left_upper_joint_msg = JointState()
        left_upper_joint_msg.header.stamp = current_time.to_msg()
        left_upper_joint_msg.name = ["left_up_wheel_link_joint"]
        left_upper_joint_msg.position = [left_wheel_position]
        left_upper_joint_msg.velocity = [lVel]

        right_upper_joint_msg = JointState()
        right_upper_joint_msg.header.stamp = current_time.to_msg()
        right_upper_joint_msg.name = ["right_up_wheel_link_joint"]
        right_upper_joint_msg.position = [right_wheel_position]
        right_upper_joint_msg.velocity = [rVel]

        left_down_joint_msg = JointState()
        left_down_joint_msg.header.stamp = current_time.to_msg()
        left_down_joint_msg.name = ["left_down_wheel_link_joint"]
        left_down_joint_msg.position = [left_wheel_position]
        left_down_joint_msg.velocity = [lVel]

        right_down_joint_msg = JointState()
        right_down_joint_msg.header.stamp = current_time.to_msg()
        right_down_joint_msg.name = ["right_down_wheel_link_joint"]
        right_down_joint_msg.position = [right_wheel_position]
        right_down_joint_msg.velocity = [rVel]

        self.joint_publisher.publish(left_upper_joint_msg)
        self.joint_publisher.publish(right_upper_joint_msg)
        self.joint_publisher.publish(left_down_joint_msg)
        self.joint_publisher.publish(right_down_joint_msg)


    def handle_default(self, json_data):
        pass

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