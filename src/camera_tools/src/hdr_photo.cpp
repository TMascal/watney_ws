// hdr_photo_server.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "camera_tools_interfaces/srv/take_picture.hpp"
#include "camera_tools_interfaces/srv/change_exposure.hpp"
#include <iostream>
#include <memory>
#include <string>

class HDRPhotoServer : public rclcpp::Node
{
public:
    HDRPhotoServer() : Node("hdr_photo_server")
    {
        // Initialize service clients
        take_picture_client_ = this->create_client<sensor_msgs::srv::Image>("/take_picture");
        change_exposure_client_ = this->create_client<example_interfaces::srv::ChangeExposure>("/change_exposure");

        // Ensure services are ready
        while (!take_picture_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the /take_picture service to be available...");
        }
        while (!change_exposure_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for the /change_exposure service to be available...");
        }

        RCLCPP_INFO(this->get_logger(), "HDRPhotoServer node started.");
        perform_hdr_capture();
    }

private:
    void perform_hdr_capture()
    {
        // Define the exposure values to create an HDR photo
        std::vector<int64_t> exposure_values = {20, 50, 100};

        for (const auto& exposure_value : exposure_values) {
            RCLCPP_INFO(this->get_logger(), "Changing exposure to: %ld", exposure_value);

            // Change exposure value
            auto exposure_request = std::make_shared<example_interfaces::srv::ChangeExposure::Request>();
            exposure_request->exposure_value = exposure_value;
            auto exposure_response = change_exposure_client_->async_send_request(exposure_request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), exposure_response) ==
                rclcpp::FutureReturnCode::SUCCESS &&
                exposure_response.get()->success) {
                RCLCPP_INFO(this->get_logger(), "Successfully changed exposure to: %ld", exposure_value);
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to change exposure to: %ld. Skipping this exposure level.", exposure_value);
                continue;
            }

            // Take a picture at the new exposure
            RCLCPP_INFO(this->get_logger(), "Taking picture at exposure: %ld", exposure_value);
            auto take_picture_request = std::make_shared<sensor_msgs::srv::Image::Request>();
            auto take_picture_response = take_picture_client_->async_send_request(take_picture_request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), take_picture_response) ==
                rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Captured an image at exposure: %ld", exposure_value);
                // Process the captured image here, if needed
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to capture an image at exposure: %ld. Skipping this exposure level.", exposure_value);
            }
        }

        RCLCPP_INFO(this->get_logger(), "HDR capture complete.");
    }

    // Service clients
    rclcpp::Client<example_interfaces::srv::ChangeExposure>::SharedPtr change_exposure_client_;
    rclcpp::Client<sensor_msgs::srv::Image>::SharedPtr take_picture_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HDRPhotoServer>());
    rclcpp::shutdown();
    return 0;
}