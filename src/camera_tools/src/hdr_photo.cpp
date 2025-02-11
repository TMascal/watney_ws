#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "camera_tools_interfaces/srv/change_exposure.hpp"

class HDRPhotoServer : public rclcpp::Node
{
public:
    HDRPhotoServer() : Node("hdr_photo_server")
    {
        RCLCPP_INFO(this->get_logger(), "Starting hdr_photo_server node...");

        // Create a client for the "change_exposure" service
        client_ = this->create_client<camera_tools_interfaces::srv::ChangeExposure>("change_exposure");

        // Example: Call the changeExposure service during initialization
        int exposure_value = 1000;
        if (callChangeExposure(exposure_value))
        {
            RCLCPP_INFO(this->get_logger(), "Successfully set exposure to %.2d", exposure_value);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set exposure.");
        }
    }

    // Synchronous call to the change_exposure service
    bool callChangeExposure(float value);

private:
    rclcpp::Client<camera_tools_interfaces::srv::ChangeExposure>::SharedPtr client_;
};

// Definition of the callChangeExposure function
bool HDRPhotoServer::callChangeExposure(float value)
{
    // Wait for the service to become available
    if (!client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(this->get_logger(), "Service 'change_exposure' not available.");
        return false;
    }

    // Create a request object and set exposure value
    auto request = std::make_shared<camera_tools_interfaces::srv::ChangeExposure::Request>();
    request->exposure_value = value;

    // Send the request and wait for the response
    auto future = client_->async_send_request(request);

    // Synchronously wait for the response
    auto return_code = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

    if (return_code == rclcpp::FutureReturnCode::SUCCESS)
    {
        // Handle the response
        auto response = future.get(); // Get the response
        return response->success;    // Return whether the service call succeeded
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        return false;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HDRPhotoServer>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}