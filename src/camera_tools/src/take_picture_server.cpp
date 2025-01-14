//
// Created by Tim Mascal on 1/9/25.
//

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "camera_tools_interfaces/srv/take_picture.hpp" // Update to use your service definition

using namespace std::placeholders;

class TakePictureServiceServer : public rclcpp::Node
{
public:
    TakePictureServiceServer()
        : Node("take_picture_service_server")
    {
        service_ = this->create_service<camera_tools_interfaces::srv::TakePicture>(
            "take_picture",
            std::bind(&TakePictureServiceServer::handle_request, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Service server ready.");
    }

private:
    void handle_request(const std::shared_ptr<camera_tools_interfaces::srv::TakePicture::Request> request,
                    std::shared_ptr<camera_tools_interfaces::srv::TakePicture::Response> response)
{
    (void)request; // Acknowledge that the request was received, but it is empty, so it is ignored.

    // Video capturing logic
    cv::Mat frame;
    cv::VideoCapture cap(2); // Open the default camera (index 0)

    if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open the camera.");
        return;
    }

    cap >> frame; // Capture a single frame

    if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Captured frame is empty.");
        return;
    }

    // Convert cv::Mat to ROS2 sensor_msgs/Image
    auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    response->image = *img_msg;

    RCLCPP_INFO(this->get_logger(), "Image captured and returned.");
}

    // Declare a service Server for TakePicture Service
    rclcpp::Service<camera_tools_interfaces::srv::TakePicture>::SharedPtr service_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TakePictureServiceServer>());
    rclcpp::shutdown();
    return 0;
}