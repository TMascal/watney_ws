import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistWithCovarianceStamped
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

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.read_publisher = self.create_publisher(String, '/h2l_node/read', 10)
        self.write_subscription = self.create_subscription(String, '/h2l_node/write', self.write_serial, 10)
        self.imu_publisher = self.create_publisher(Imu, '/h2l_node/imu/raw', 10)
        self.mag_publisher = self.create_publisher(MagneticField, '/h2l_node/imu/mag', 10)
        self.twist_publisher = self.create_publisher(TwistWithCovarianceStamped, '/h2l_node/wheel_velocity', 10)
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_subscription = self.create_subscription(Twist, '/h2l_node/cmd_vel', self.handle_cmd_vel, 10)
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

        self.get_logger().info(f"Command Executed. Defualt Values Initialized. There you are. Deploying!")

    def read_serial(self):
        feedback_rate_set = False
        while rclpy.ok():
            data = self.ser.readline().decode('utf-8')
            if data:
                msg = String()
                msg.data = data
                self.read_publisher.publish(msg)
                self.handle_json(data)
                if not feedback_rate_set:
                    self.set_feedback_rate(25)
                    self.get_logger().info("Feedback frequency set to 25 Hz")
                    feedback_rate_set = True

    def set_feedback_rate(self, rate_hz):
        json_data = {
            "T": 142,
            "cmd": int(1000 / rate_hz)  # Convert Hz to milliseconds
        }
        json_str = json.dumps(json_data)
        self.write_serial(String(data=json_str))

    def handle_json(self, data):
        try:
            json_data = json.loads(data)
            if 'T' in json_data:
                t_value = json_data['T']
                handlers = {
                    1001: self.handle_1001,
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

    def handle_1001(self, json_data):
        current_time = self.get_clock().now()
        self.last_time = current_time

        lVel = float(json_data.get('L', 0.0))
        rVel = float(json_data.get('R', 0.0))
        odl = float(json_data.get('odl', 0.0))/100 # Convert to meters
        odr = float(json_data.get('odr', 0.0))/100 # Convert to meters
        width = 0.172
        wheel_radius = .08

        delta_odl = odl - self.previous_odl
        delta_odr = odr - self.previous_odr
        left_wheel_position = delta_odl / wheel_radius
        right_wheel_position = delta_odr / wheel_radius
        self.previous_odl = odl
        self.previous_odr = odr

        linear_velocity_x = (rVel + lVel) / 2
        angular_velocity_z = (rVel - lVel) / width

        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = "base_link"
        imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        imu_msg.angular_velocity.x = float(json_data.get('gx', 0.0)) * (math.pi / 180.0)
        imu_msg.angular_velocity.y = float(json_data.get('gy', 0.0)) * (math.pi / 180.0)
        imu_msg.angular_velocity.z = float(json_data.get('gz', 0.0)) * (math.pi / 180.0)
        imu_msg.angular_velocity_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02] # Filler Values

        imu_msg.linear_acceleration.x = float(json_data.get('ax', 0.0)) / 10000 * 9.81
        imu_msg.linear_acceleration.y = float(json_data.get('ay', 0.0)) / 10000 * 9.81
        imu_msg.linear_acceleration.z = float(json_data.get('az', 0.0)) / 10000 * 9.81
        imu_msg.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]  # Filler Values

        self.imu_publisher.publish(imu_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp = current_time.to_msg()
        mag_msg.header.frame_id = "base_link"
        mag_msg.magnetic_field.x = float(json_data.get('mx', 0.0))
        mag_msg.magnetic_field.y = float(json_data.get('my', 0.0))
        mag_msg.magnetic_field.z = float(json_data.get('mz', 0.0))
        mag_msg.magnetic_field_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]  # Filler Values

        self.mag_publisher.publish(mag_msg)

        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = current_time.to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.twist.linear.x = linear_velocity_x
        twist_msg.twist.twist.angular.z = angular_velocity_z
        twist_msg.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,   # Covariance for linear X
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,   # Covariance for linear Y (ignored)
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,   # Covariance for linear Z (ignored)
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,   # Covariance for angular X (ignored)
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,   # Covariance for angular Y (ignored)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05   # Covariance for yaw rate (tune as needed)
        ]

        self.twist_publisher.publish(twist_msg)

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