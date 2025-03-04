//
// Create by Tim Mascal on 1/9/25.
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

        // Video capturing logic
        cap.open(2, cv::CAP_V4L2); // Open the default camera (index 0)

        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the camera.");
            return;
        }

        // Set the desired resolution
        int desired_width = 2592;  // Hardcoded width
        int desired_height = 1944; // Hardcoded height
        cap.set(cv::CAP_PROP_FRAME_WIDTH, desired_width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, desired_height);
        // Attempt to enable camera auto-focus
        if (cap.set(cv::CAP_PROP_AUTOFOCUS, 1)) {
            RCLCPP_INFO(this->get_logger(), "Auto-focus enabled.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Auto-focus not supported by the camera.");
        }
    }

private:
    void handle_request(const std::shared_ptr<camera_tools_interfaces::srv::TakePicture::Request> request,
                    std::shared_ptr<camera_tools_interfaces::srv::TakePicture::Response> response)
{
    (void)request;
    // Discard initial frames to ensure the captured frame is focused
    for (int i = 0; i < 5; ++i) {
        cap >> frame;
    }

    // Delay to allow the camera's auto-focus to adjust
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    cap >> frame; // Capture and discard frame

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

    // Declare member variables for video capturing and frame storage
    cv::VideoCapture cap;
    cv::Mat frame;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TakePictureServiceServer>());
    rclcpp::shutdown();
    return 0;
}