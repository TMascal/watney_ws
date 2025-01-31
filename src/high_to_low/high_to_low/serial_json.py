import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
import serial
import threading
import argparse
import json
import math
import tf_transformations

class SerialNode(Node):
    def __init__(self, port, baudrate):
        super().__init__('h2l_node')
        self.read_publisher = self.create_publisher(String, '/h2l_node/read', 10)
        self.write_subscription = self.create_subscription(String, '/h2l_node/write', self.write_serial, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/h2l_node/odom', 10)
        self.cmd_vel_subscription = self.create_subscription(Twist, '/h2l_node/cmd_vel', self.handle_cmd_vel, 10)
        self.write_subscription  # prevent unused variable warning
        self.ser = serial.Serial(port, baudrate, dsrdtr=None)
        self.ser.setRTS(False)
        self.ser.setDTR(False)
        self.serial_recv_thread = threading.Thread(target=self.read_serial)
        self.serial_recv_thread.daemon = True
        self.serial_recv_thread.start()

        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_linear_velocity = 0.0
        self.prev_angular_velocity = 0.0
        self.prev_roll = 0.0
        self.prev_pitch = 0.0
        self.prev_yaw = 0.0

        self.get_logger().info(f"Connection Complete. Values Initialized. There you are. Deploying!")

    def read_serial(self):
        while rclpy.ok():
            data = self.ser.readline().decode('utf-8')
            if data:
                # self.get_logger().info(f"Received: {data}")
                msg = String()
                msg.data = data
                self.read_publisher.publish(msg)
                self.handle_json(data)

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
        # self.get_logger().info("Handling T=1001")

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        lVel = float(json_data.get('L', 0.0))
        rVel = float(json_data.get('R', 0.0))
        width = 0.172

        linear_velocity = (rVel + lVel) / 2
        angular_velocity = (rVel - lVel) / width

        # Trapezoidal rule for integration
        self.theta += (self.prev_angular_velocity + angular_velocity) / 2 * dt
        self.x += (self.prev_linear_velocity + linear_velocity) / 2 * math.cos(self.theta) * dt
        self.y += (self.prev_linear_velocity + linear_velocity) / 2 * math.sin(self.theta) * dt

        self.prev_linear_velocity = linear_velocity
        self.prev_angular_velocity = angular_velocity

        # Double check units for these if values seem wrong
        roll = (self.prev_roll + float(json_data.get('gx', 0.0))) / 2 * dt
        pitch = (self.prev_pitch + float(json_data.get('gy', 0.0))) / 2 * dt
        yaw = (self.prev_yaw + float(json_data.get('gz', 0.0))) / 2 * dt

        self.prev_roll = roll
        self.prev_pitch = pitch
        self.prev_yaw = yaw

        quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        qx, qy, qz, qw = quaternion

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation = Quaternion(
            x=qx,
            y=qy,
            z=qz,
            w=qw
        )

        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        self.odom_publisher.publish(odom_msg)

    def handle_default(self, json_data):
        pass
        #self.get_logger().error("T value not in dictionary")
        # There should always be a value for T

    def write_serial(self, msg):
        self.ser.write(msg.data.encode() + b'\n')

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Serial JSON Communication')
    parser.add_argument('port', type=str, nargs='?', default='/dev/serial0', help='Serial port name (e.g., COM1 or /dev/ttyUSB0)') # /dev/serial0 for RPi 4 UART pins
    parser.add_argument('baudrate', type=int, nargs='?', default=115200, help='Serial baudrate (e.g., 9600 or 115200)') # 115200 default baudrate for UGV 6x4
    args = parser.parse_args()
    serial_node = SerialNode(args.port, args.baudrate)
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()