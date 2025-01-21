import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import serial
import threading
import argparse
import json

class SerialNode(Node):
    def __init__(self, port, baudrate):
        super().__init__('high_to_low_serial_node')
        self.publisher_ = self.create_publisher(String, '/ugv/read', 10) #continuosly publish to read
        self.subscription = self.create_subscription(String, '/ugv/write', self.write_serial, 10) # setup callback to write_serial
        self.imu_publisher = self.create_publisher(Imu, '/ugv/imu', 10) # publish imu data to /ugv/imu topic
        self.subscription  # prevent unused variable warning
        self.ser = serial.Serial(port, baudrate, dsrdtr=None)
        self.ser.setRTS(False)
        self.ser.setDTR(False)
        self.serial_recv_thread = threading.Thread(target=self.read_serial)
        self.serial_recv_thread.daemon = True
        self.serial_recv_thread.start()

    def read_serial(self):
        while rclpy.ok():
            data = self.ser.readline().decode('utf-8')
            if data:
                self.get_logger().info(f"Received: {data}")
                self.handle_json(data)

    def handle_json(self, data):
        try:
            json_data = json.loads(data)
            #self.get_logger().info(f"Parsed JSON: {json_data}")
            
            if 'T' in json_data:
                t_value = json_data['T']
                handlers = {
                    1002: self.handle_1002,
                    # Add more handlers as needed
                }
                handler = handlers.get(t_value, self.handle_default)
                handler(json_data)
            else:
                self.get_logger().warning("No 'T' value found in JSON")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e}")

    def handle_1002(self, json_data):
        self.get_logger().info("Handling T=1002")
        imu_msg = Imu()
            
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_frame"
            
        imu_msg.orientation.x = 0.0  # Find a way to populate these. Could use a service that requests a few different messages.
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0
            
        imu_msg.angular_velocity.x = json_data.get('gx', 0.0)
        imu_msg.angular_velocity.y = json_data.get('gy', 0.0)
        imu_msg.angular_velocity.z = json_data.get('gz', 0.0)
            
        imu_msg.linear_acceleration.x = json_data.get('ax', 0.0)
        imu_msg.linear_acceleration.y = json_data.get('ay', 0.0)
        imu_msg.linear_acceleration.z = json_data.get('az', 0.0)
            
        self.imu_publisher.publish(imu_msg)
        #self.get_logger().info(f"Published IMU data: {imu_msg}")

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