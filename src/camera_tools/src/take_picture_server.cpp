//
// Created by Tim Mascal on 1/9/25.
//

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "camera_tools_interfaces/srv/take_picture.hpp" // Update to use your service definition
#include "camera_tools_interfaces/srv/change_exposure.hpp" // Update to use your service definition"

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

        // Initialize the Change Exposure client
        change_exposure_client_ = this->create_client<camera_tools_interfaces::srv::ChangeExposure>("change_exposure");
        while (!change_exposure_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for the ChangeExposure service to become available...");
        }

    }

private:
    void handle_request(const std::shared_ptr<camera_tools_interfaces::srv::TakePicture::Request> request,
                    std::shared_ptr<camera_tools_interfaces::srv::TakePicture::Response> response)
{
    (void)request; // Acknowledge that the request was received, but it is empty, so it is ignored.

    // Define the request to change exposure
    auto exposure_request = std::make_shared<camera_tools_interfaces::srv::ChangeExposure::Request>();
    exposure_request->exposure_value = 100; // Example exposure value (modify as needed).

    // Call the change_exposure service
    RCLCPP_INFO(this->get_logger(), "Calling change_exposure service...");
    if (!change_exposure_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "ChangeExposure service not available. Aborting picture capture.");
        response->image = sensor_msgs::msg::Image(); // Return an empty image message
        return;
    }

    auto future = change_exposure_client_->async_send_request(exposure_request);

    // Use a synchronous wait for the result
    auto response_received = future.wait_for(std::chrono::seconds(5));
    if (response_received != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call change_exposure service. Aborting picture capture.");
        response->image = sensor_msgs::msg::Image(); // Return an empty image message
        return;
    }

    auto exposure_response = future.get();
    if (!exposure_response->success) {
        RCLCPP_ERROR(this->get_logger(), "ChangeExposure service returned failure. Aborting picture capture.");
        response->image = sensor_msgs::msg::Image(); // Return an empty image message
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Exposure successfully modified. Proceeding to capture image.");

    // Video capturing logic
    cv::Mat frame;
    cv::VideoCapture cap(0); // Open the default camera (index 0)

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

    // Declare a service Client for ChangeExposure Service
    rclcpp::Client<camera_tools_interfaces::srv::ChangeExposure>::SharedPtr change_exposure_client_;




};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TakePictureServiceServer>());
    rclcpp::shutdown();
    return 0;
}