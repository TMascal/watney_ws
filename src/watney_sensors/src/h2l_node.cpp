#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "serialib.h"               // Using serialib from imabot2/serialib
#include <nlohmann/json.hpp>         // For JSON parsing
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <chrono>
#include <cmath>

using json = nlohmann::json;
using namespace std::chrono_literals;

class SerialNode : public rclcpp::Node
{
public:
  SerialNode()
  : Node("h2l_node"),
    keep_running_(true),
    calibration_event_(false),
    previous_odl_m_(0.0),
    previous_odr_m_(0.0),
    x_position_(0.0),
    y_position_(0.0),
    theta_(0.0)
  {
    // Declare and get parameters
    this->declare_parameter<std::string>("port", "/dev/serial0");
    this->declare_parameter<int>("baudrate", 115200);
    this->declare_parameter<bool>("use_mag", false);
    this->declare_parameter<double>("feedback_frequency", 50.0);

    port_     = this->get_parameter("port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    use_mag_  = this->get_parameter("use_mag").as_bool();
    double feedback_freq = this->get_parameter("feedback_frequency").as_double();

    last_cmd_vel_time_ = this->get_clock()->now();

    // Create publishers
    read_publisher_    = this->create_publisher<std_msgs::msg::String>("/h2l_node/read", 10);
    chatter_publisher_ = this->create_publisher<std_msgs::msg::String>("/h2l_node/chatter", 10);
    imu_publisher_     = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
    mag_publisher_     = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
    odom_publisher_    = this->create_publisher<nav_msgs::msg::Odometry>("/h2l_node/wheel_odometry", 10);
    joint_publisher_   = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Create subscriptions
    write_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/h2l_node/write", 10,
      std::bind(&SerialNode::writeSerialCallback, this, std::placeholders::_1));
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&SerialNode::handleCmdVel, this, std::placeholders::_1));

    // Initialize serial connection using serialib
    int result = serial_port_.openDevice(port_.c_str(), baudrate_);
    if (result != 1)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s at %d baud (error code: %d)", port_.c_str(), baudrate_, result);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %d baud", port_.c_str(), baudrate_);
      // If necessary, set RTS/DTR here (serialib may not provide these directly)
    }

    // Start the serial receiving thread
    serial_recv_thread_ = std::thread(&SerialNode::readSerial, this);

    // Set an initial (fast) feedback rate then calibrate the IMU before switching to the desired frequency
    setFeedbackRate(500.0);
    hlCalibrateImu();
    setFeedbackRate(feedback_freq);
    RCLCPP_INFO(this->get_logger(), "Feedback frequency set to %f Hz", feedback_freq);

    // Timer for velocity timeout
    velocity_timeout_timer_ = this->create_wall_timer(
      100ms, std::bind(&SerialNode::checkVelocityTimeout, this));

    RCLCPP_INFO(this->get_logger(), "Command executed. Default values initialized. There you are. Deploying!");
  }

  ~SerialNode()
  {
    keep_running_ = false;
    if (serial_recv_thread_.joinable()) {
      serial_recv_thread_.join();
    }
    serial_port_.closeDevice();
  }

private:
  // Parameters
  std::string port_;
  int baudrate_;
  bool use_mag_;

  // serialib instance and thread control
  serialib serial_port_;
  std::thread serial_recv_thread_;
  std::atomic<bool> keep_running_;

  // Calibration
  std::atomic<bool> calibration_event_;
  std::mutex calib_mutex_;
  std::vector<std::string> calibration_data_;

  // IMU Offsets
  double accel_offsets_[3] = {0.0, 0.0, 0.0};
  double gyro_offsets_[3]  = {0.0, 0.0, 0.0};

  // Publishers and subscriptions
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr read_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr write_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

  // Timer and time tracking for velocity timeout
  rclcpp::TimerBase::SharedPtr velocity_timeout_timer_;
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time last_time_;

  // Odometry and positioning variables
  double previous_odl_m_;
  double previous_odr_m_;
  double x_position_;
  double y_position_;
  double theta_;

  // Thread function to continuously read the serial port
  void readSerial()
{
  while (rclcpp::ok() && keep_running_) {
    char buffer[1024];
    // Note the reordering: stop character '\n' comes second.
    int n = serial_port_.readString(buffer, '\n', 1023, 1000);
    if (n > 0) {
      buffer[n] = '\0';
      std::string data(buffer);
      std_msgs::msg::String msg;
      msg.data = data;
      read_publisher_->publish(msg);

      if (!calibration_event_) {
        std::lock_guard<std::mutex> lock(calib_mutex_);
        calibration_data_.push_back(data);
      } else {
        handleJson(data);
      }
    } else if (n < 0) {
      RCLCPP_WARN(this->get_logger(), "Error reading serial data (error code: %d)", n);
      serial_port_.flushReceiver();
    }
  }
}


  // Callback for write subscription (send message to serial)
  void writeSerialCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    writeSerial(msg->data);
  }

  // Write a string to the serial port using serialib
  void writeSerial(const std::string & data)
  {
    if (serial_port_.isDeviceOpen()) {
      int ret = serial_port_.writeString((data + "\n").c_str());
      if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error writing to serial (error code: %d)", ret);
      }
    }
  }

  // Set the feedback rate (convert Hz to milliseconds for the target device)
  void setFeedbackRate(double rate_hz)
  {
    json json_data;
    json_data["T"]   = 142;
    json_data["cmd"] = static_cast<int>(1000.0 / rate_hz);
    writeSerial(json_data.dump());
  }

  // IMU calibration: Collect a number of samples and compute offsets
  void hlCalibrateImu(int num_samples = 1000)
  {
    RCLCPP_INFO(this->get_logger(), "Starting IMU calibration... Keep sensor flat and steady!");
    double ax = 0.0, ay = 0.0, az = 0.0;
    double gx = 0.0, gy = 0.0, gz = 0.0;

    while (rclcpp::ok() && static_cast<int>(calibration_data_.size()) < num_samples) {
      std::this_thread::sleep_for(100ms);
    }

    for (int i = 0; i < num_samples; ++i) {
      try {
        auto json_data = json::parse(calibration_data_[i]);
        ax += json_data.value("ax", 0.0);
        ay += json_data.value("ay", 0.0);
        az += json_data.value("az", 0.0);
        gx += json_data.value("gx", 0.0);
        gy += json_data.value("gy", 0.0);
        gz += json_data.value("gz", 0.0);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Error during calibration: %s", e.what());
        continue;
      }
    }

    accel_offsets_[0] = ax / num_samples;
    accel_offsets_[1] = ay / num_samples;
    accel_offsets_[2] = (az / num_samples) - 8192.0;  // Adjust to 1g

    gyro_offsets_[0] = gx / num_samples;
    gyro_offsets_[1] = gy / num_samples;
    gyro_offsets_[2] = gz / num_samples;

    RCLCPP_INFO(this->get_logger(), "IMU Calibration complete!");
    RCLCPP_INFO(this->get_logger(), "Accel Offsets: x=%f, y=%f, z=%f", accel_offsets_[0], accel_offsets_[1], accel_offsets_[2]);
    RCLCPP_INFO(this->get_logger(), "Gyro Offsets: x=%f, y=%f, z=%f", gyro_offsets_[0], gyro_offsets_[1], gyro_offsets_[2]);

    calibration_event_ = true;
  }

  // Process JSON data received from the serial port
  void handleJson(const std::string & data)
  {
    try {
      auto json_data = json::parse(data);
      if (json_data.contains("T")) {
        int t_val = json_data["T"];
        if (t_val == 1001) {
          handle1001(json_data);
        } else {
          handleDefault(json_data);
        }
      } else {
        std_msgs::msg::String chatter_msg;
        chatter_msg.data = data;
        chatter_publisher_->publish(chatter_msg);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to decode JSON: %s", e.what());
      std_msgs::msg::String chatter_msg;
      chatter_msg.data = data;
      chatter_publisher_->publish(chatter_msg);
    }
  }

  // Default handler for unrecognized JSON messages
  void handleDefault(const json & json_data)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received message with T: %d", json_data.value("T", -1));
  }

  // Callback for cmd_vel messages; send a JSON command over serial
  void handleCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_vel_time_ = this->get_clock()->now();
    json json_data;
    json_data["T"] = 13;
    json_data["X"] = msg->linear.x;
    json_data["Z"] = msg->angular.z;
    writeSerial(json_data.dump());
  }

  // Timer callback: if no cmd_vel received for 3 seconds, send zero velocity
  void checkVelocityTimeout()
  {
    auto current_time = this->get_clock()->now();
    double time_since = (current_time - last_cmd_vel_time_).nanoseconds() / 1e9;
    if (time_since > 3.0) {
      RCLCPP_INFO(this->get_logger(), "Velocity timeout detected. Sending zero velocity.");
      auto zero_twist = std::make_shared<geometry_msgs::msg::Twist>();
      handleCmdVel(zero_twist);
      // Reset last_cmd_vel_time_ using the current time, not a zero time.
      last_cmd_vel_time_ = current_time;
    }
  }

  // Handler for T==1001 JSON messages: process and publish IMU, odometry, joint states, and magnetometer data.
  void handle1001(const json & json_data)
  {
    auto now = this->get_clock()->now();
    last_time_ = now;
    double accel_ssf = 8192.0;
    double gyro_ssf  = 32.8;
    double magn_ssf  = 0.15;

    // IMU message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = "imu_link";
    if (!use_mag_) {
      imu_msg.orientation_covariance = {0.0028, -0.0001, 0.0033,
                                          -0.0001, 0.0001, 0.0010,
                                          0.0033, 0.0010, 8.3553};
    } else {
      imu_msg.orientation_covariance = {0.0016, -0.0000, 0.0008,
                                          -0.0000, 0.0000, -0.0009,
                                          0.0008, -0.0009, 7.2158};
    }
    imu_msg.angular_velocity.x = ((json_data.value("gx", 0.0) - gyro_offsets_[0]) / gyro_ssf) * (M_PI / 180.0);
    imu_msg.angular_velocity.y = ((json_data.value("gy", 0.0) - gyro_offsets_[1]) / gyro_ssf) * (M_PI / 180.0);
    imu_msg.angular_velocity.z = ((json_data.value("gz", 0.0) - gyro_offsets_[2]) / gyro_ssf) * (M_PI / 180.0);
    imu_msg.angular_velocity_covariance = {0.5669e-06, 0.0285e-06, -0.0037e-06,
                                             0.0285e-06, 0.5614e-06, 0.0141e-06,
                                             -0.0037e-06, 0.0141e-06, 0.4967e-06};
    imu_msg.linear_acceleration.x = (json_data.value("ax", 0.0) - accel_offsets_[0]) / accel_ssf * 9.81;
    imu_msg.linear_acceleration.y = (json_data.value("ay", 0.0) - accel_offsets_[1]) / accel_ssf * 9.81;
    imu_msg.linear_acceleration.z = (json_data.value("az", 0.0) - accel_offsets_[2]) / accel_ssf * 9.81;
    imu_msg.linear_acceleration_covariance = {0.0015, 0.0000, -0.0000,
                                                0.0000, 0.0014, 0.0000,
                                               -0.0000, 0.0000, 0.0018};
    imu_publisher_->publish(imu_msg);

    // Magnetic Field message
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.stamp = now;
    mag_msg.header.frame_id = "imu_link";
    mag_msg.magnetic_field.x = json_data.value("mx", 0.0) * magn_ssf;
    mag_msg.magnetic_field.y = json_data.value("my", 0.0) * magn_ssf;
    mag_msg.magnetic_field.z = json_data.value("mz", 0.0) * magn_ssf;
    mag_msg.magnetic_field_covariance = {0.9115, -0.1343, -0.2316,
                                           -0.1343,  0.6865,  0.0859,
                                           -0.2316,  0.0859,  0.7230};
    mag_publisher_->publish(mag_msg);

    // Odometry and Joint State Calculations
    double width = 0.172;
    double wheel_diameter = 0.08;

    double lVel   = json_data.value("L", 0.0);
    double rVel   = json_data.value("R", 0.0);
    double odl_cm = json_data.value("odl", 0.0);
    double odr_cm = json_data.value("odr", 0.0);

    double odl_m = odl_cm / 100.0;
    double odr_m = odr_cm / 100.0;

    double left_wheel_position  = 2.0 * odl_m / wheel_diameter;
    double right_wheel_position = 2.0 * odr_m / wheel_diameter;
    double left_wheel_velocity  = 2.0 * lVel / wheel_diameter;
    double right_wheel_velocity = 2.0 * rVel / wheel_diameter;

    double linear_velocity_x = (rVel + lVel) / 2.0;
    double angular_velocity_z = (rVel - lVel) / width;
    double delta_d = 0.0;
    double delta_theta = 0.0;
    static bool odometry_initialized = false;
    if (!odometry_initialized) {
      previous_odl_m_ = odl_m;
      previous_odr_m_ = odr_m;
      odometry_initialized = true;
    } else {
      delta_d     = ((odl_m - previous_odl_m_) + (odr_m - previous_odr_m_)) / 2.0;
      delta_theta = ((odr_m - previous_odr_m_) - (odl_m - previous_odl_m_)) / width;
      previous_odl_m_ = odl_m;
      previous_odr_m_ = odr_m;
    }
    double scale_factor = 3.5;
    x_position_ += delta_d * scale_factor * std::cos(theta_ + delta_theta / 2.0);
    y_position_ += delta_d * scale_factor * std::sin(theta_ + delta_theta / 2.0);
    theta_ += delta_theta;

    // Covariance values (as per your original node)
    std::array<double,36> odom_pose_cov = {{
        0.02,    0.0,     0.0,     0.0,     0.0,     0.0,
        0.0,     0.02,    0.0,     0.0,     0.0,     0.0,
        0.0,     0.0,  99999.0,    0.0,     0.0,     0.0,
        0.0,     0.0,     0.0,  99999.0,    0.0,     0.0,
        0.0,     0.0,     0.0,     0.0,  99999.0,    0.0,
        0.0,     0.0,     0.0,     0.0,     0.0,     0.2
      }};
      
    std::array<double,36> odom_twist_cov = {{
        0.01,    0.0,     0.0,     0.0,     0.0,     0.0,
        0.0,     0.01,    0.0,     0.0,     0.0,     0.0,
        0.0,     0.0,  99999.0,    0.0,     0.0,     0.0,
        0.0,     0.0,     0.0,  99999.0,    0.0,     0.0,
        0.0,     0.0,     0.0,     0.0,  99999.0,    0.0,
        0.0,     0.0,     0.0,     0.0,     0.0,     0.2
      }};

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x = x_position_;
    odom_msg.pose.pose.position.y = y_position_;
    odom_msg.pose.pose.orientation.z = std::sin(theta_ / 2.0);
    odom_msg.pose.pose.orientation.w = std::cos(theta_ / 2.0);
    odom_msg.twist.twist.linear.x = linear_velocity_x;
    odom_msg.twist.twist.angular.z = angular_velocity_z;
    odom_msg.pose.covariance = odom_pose_cov;
    odom_msg.twist.covariance = odom_twist_cov;
    odom_publisher_->publish(odom_msg);

    // Publish Joint States
    sensor_msgs::msg::JointState left_upper_joint_msg;
    left_upper_joint_msg.header.stamp = now;
    left_upper_joint_msg.name = {"left_up_wheel_link_joint"};
    left_upper_joint_msg.position = {left_wheel_position};
    left_upper_joint_msg.velocity = {left_wheel_velocity};

    sensor_msgs::msg::JointState right_upper_joint_msg;
    right_upper_joint_msg.header.stamp = now;
    right_upper_joint_msg.name = {"right_up_wheel_link_joint"};
    right_upper_joint_msg.position = {right_wheel_position};
    right_upper_joint_msg.velocity = {right_wheel_velocity};

    sensor_msgs::msg::JointState left_down_joint_msg;
    left_down_joint_msg.header.stamp = now;
    left_down_joint_msg.name = {"left_down_wheel_link_joint"};
    left_down_joint_msg.position = {left_wheel_position};
    left_down_joint_msg.velocity = {left_wheel_velocity};

    sensor_msgs::msg::JointState right_down_joint_msg;
    right_down_joint_msg.header.stamp = now;
    right_down_joint_msg.name = {"right_down_wheel_link_joint"};
    right_down_joint_msg.position = {right_wheel_position};
    right_down_joint_msg.velocity = {right_wheel_velocity};

    joint_publisher_->publish(left_upper_joint_msg);
    joint_publisher_->publish(right_upper_joint_msg);
    joint_publisher_->publish(left_down_joint_msg);
    joint_publisher_->publish(right_down_joint_msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto serial_node = std::make_shared<SerialNode>();
  rclcpp::spin(serial_node);
  rclcpp::shutdown();
  return 0;
}
