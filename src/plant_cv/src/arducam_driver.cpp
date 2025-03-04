//
// Created by Tim Mascal on 12/3/24.
//
// The purpose of this file is to interface with the ArduCam with ROS


#include <opencv2/opencv.hpp>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("image_publisher_node");
    auto publisher = node->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Could not open camera.");
        return -1;
    }

    int desired_width = 2592;  // Hardcoded width
    int desired_height = 1944; // Hardcoded height
    cap.set(cv::CAP_PROP_FRAME_WIDTH, desired_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, desired_height);

    cv::Mat frame;
    while (rclcpp::ok()) {
        cap >> frame;
        if (frame.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Could not grab a frame.");
            break;
        }

        // Query the current exposure
        double exposure = cap.get(cv::CAP_PROP_EXPOSURE);
        RCLCPP_INFO(node->get_logger(), "Exposure time: %f", exposure);

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher->publish(*msg);
        rclcpp::spin_some(node);
    }

    cap.release();
    rclcpp::shutdown();
    return 0;
}