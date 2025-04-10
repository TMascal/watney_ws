//
// Created by TMascal on 12/3/24.
//
// Script meant to subscribe to ROS Topic With Camera Data and display it in a window

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraViewer : public rclcpp::Node {
public:
    CameraViewer()
    : Node("camera_viewer") {
        // Create the subscription to the image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            std::bind(&CameraViewer::imageCallback, this, std::placeholders::_1)
        );
        cv::namedWindow("Camera Viewer", cv::WINDOW_AUTOSIZE);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS image to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::imshow("Camera Viewer", cv_ptr->image);
            cv::waitKey(10);  // Wait for a key press for the duration specified (in milliseconds)
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraViewer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
