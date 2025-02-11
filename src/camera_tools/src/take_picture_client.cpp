#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem> // For handling directories
#include "camera_tools_interfaces/srv/take_picture.hpp"

using namespace std::chrono_literals;

class TakePictureClient : public rclcpp::Node
{
public:
    TakePictureClient()
        : Node("take_picture_client")
    {
        RCLCPP_INFO(this->get_logger(), "Starting TakePictureClient node...");

        // Create a client for the "take_picture" service
        client_ = this->create_client<camera_tools_interfaces::srv::TakePicture>("take_picture");

        RCLCPP_INFO(this->get_logger(), "Waiting for the take_picture service to become available...");
        // Wait for the service to be available
        while (!client_->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_WARN(this->get_logger(), "Service not available, waiting...");
        }

        RCLCPP_INFO(this->get_logger(), "Service is available. Sending a request...");
        send_request();
    }

private:
    void send_request()
    {
        RCLCPP_DEBUG(this->get_logger(), "Creating request object for take_picture service.");
        // Create a request object
        auto request = std::make_shared<camera_tools_interfaces::srv::TakePicture::Request>();

        // Call the service asynchronously and wait for the result in a non-blocking manner
        auto future_result = client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Waiting for the server response...");

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, 5s) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future_result.get();
            RCLCPP_INFO(this->get_logger(), "Response received from the server.");
            save_image(response->image);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive response from the server within timeout.");
        }
    }

    void save_image(const sensor_msgs::msg::Image &image_msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received image from service, preparing to save.");
        try {
            // Convert the ROS2 Image message to an OpenCV Image (cv::Mat)
            RCLCPP_DEBUG(this->get_logger(), "Converting sensor_msgs::msg::Image to cv::Mat using cv_bridge.");
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
            cv::Mat frame = cv_ptr->image;

            if (frame.empty()) {
                RCLCPP_ERROR(this->get_logger(), "The received image is empty and cannot be saved.");
                return;
            }

            // Define the target directory and file path
            std::string home = std::getenv("HOME") ? std::getenv("HOME") : "";
            if (home.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Cannot access the HOME directory.");
                return;
            }

            std::string pictures_path = home + "/Pictures";
            RCLCPP_DEBUG(this->get_logger(), "Pictures path resolved to: %s", pictures_path.c_str());

            // Ensure the Pictures directory exists
            if (!std::filesystem::exists(pictures_path)) {
                RCLCPP_INFO(this->get_logger(), "Pictures directory does not exist. Creating...");
                if (!std::filesystem::create_directory(pictures_path)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create Pictures directory at: %s", pictures_path.c_str());
                    return;
                }
            }

            // Create the filename with a timestamp
            auto time_now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            std::ostringstream oss;
            oss << std::put_time(std::localtime(&time_now), "%Y%m%d_%H%M%S"); // Format timestamp
            std::string filename = pictures_path + "/captured_" + oss.str() + ".png";

            RCLCPP_INFO(this->get_logger(), "Saving image to file: %s", filename.c_str());

            // Save the image to the file
            if (cv::imwrite(filename, frame)) {
                RCLCPP_INFO(this->get_logger(), "Image saved successfully to: %s", filename.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to save the image to the specified file.");
            }
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge exception: %s", e.what());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception caught while saving image: %s", e.what());
        }
    }

    // Declare a client for the TakePicture service
    rclcpp::Client<camera_tools_interfaces::srv::TakePicture>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing TakePictureClient...");
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<TakePictureClient>());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception caught in main: %s", e.what());
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down TakePictureClient.");
    return EXIT_SUCCESS;
}