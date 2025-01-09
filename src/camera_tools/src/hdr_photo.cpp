//
// Created by Tim Mascal on 1/8/25.
//

#include <rclcpp/rclcpp.hpp>           // ROS 2 Core Header
#include <sensor_msgs/msg/image.hpp>  // Message type for images
#include <cv_bridge/cv_bridge.h>      // cv_bridge for conversion between ROS and OpenCV
#include <opencv2/opencv.hpp>         // OpenCV for image manipulation

class ImageSubscriber : public rclcpp::Node // Extend the Node class
{
public:
    ImageSubscriber()
        : Node("hdr_image_subscriber"), image_counter_(0) // Node name and initialize counter
    {
        // Create the subscriber for the topic with the callback
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ImageSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    // Callback function for processing incoming image messages
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received an image:");
        RCLCPP_INFO(this->get_logger(), "Width: %d, Height: %d, Encoding: %s",
                    msg->width, msg->height, msg->encoding.c_str());

        try
        {
            // Convert ROS image message to OpenCV Mat using cv_bridge
            cv::Mat cv_image = cv_bridge::toCvCopy(msg, msg->encoding)->image;

            // Save the image into one of the three cv::Mat variables
            if (image_counter_ == 0)
            {
                image1_ = cv_image;
            }
            else if (image_counter_ == 1)
            {
                image2_ = cv_image;
            }
            else if (image_counter_ == 2)
            {
                image3_ = cv_image;
            }

            // Update counter to cycle between 0, 1, and 2
            image_counter_ = (image_counter_ + 1) % 3;

            RCLCPP_INFO(this->get_logger(), "Image saved to slot %d.", image_counter_);
        }
        catch (const cv_bridge::Exception &e)
        {
            // Log an error if the conversion fails
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    // ROS 2 subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    // OpenCV Mats to store the last three images
    cv::Mat image1_, image2_, image3_;

    // Counter to keep track of which image slot to update
    int image_counter_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                                    // Initialize ROS 2
    rclcpp::spin(std::make_shared<ImageSubscriber>());           // Run the ImageSubscriber node
    rclcpp::shutdown();                                          // Shutdown ROS 2
    return 0;
}