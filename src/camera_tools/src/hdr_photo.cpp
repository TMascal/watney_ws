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
        try
        {
            // Convert ROS image message to OpenCV Mat using cv_bridge
            latest_image_ = cv_bridge::toCvCopy(msg, msg->encoding)->image;

            // Log info about the received image
            RCLCPP_INFO(this->get_logger(), "Received and converted an image.");
        }
        catch (const cv_bridge::Exception &e)
        {
            // Log an error if the conversion fails
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }


public:
    // Generate an HDR Photo
    int generate_hdr_photo()
    {
        try
        {
            // Ensure that a valid image has been received
            if (latest_image_.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No valid image data available.");
                return -1; // Return failure if no image is available
            }

            // Save the image into one of the three cv::Mat variables
            if (image_counter_ == 0)
            {
                image1_ = latest_image_;
            }
            else if (image_counter_ == 1)
            {
                image2_ = latest_image_;
            }
            else if (image_counter_ == 2)
            {
                image3_ = latest_image_;
            }

            // Update counter to cycle between 0, 1, and 2
            image_counter_ = (image_counter_ + 1) % 3;

            RCLCPP_INFO(this->get_logger(), "Image saved to slot %d.", image_counter_);
        }
        catch (const std::exception &e)
        {
            // Log any generic errors
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }

        return 0; // Indicate success
    }


    // ROS 2 subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    // Placeholder for the latest image
    cv::Mat latest_image_;

    // OpenCV Mats to store the last three images
    cv::Mat image1_, image2_, image3_;

    // Counter to keep track of which image slot to update
    int image_counter_;
};

int main(int argc, char **argv) {
    // Initialize the ROS 2 framework
    rclcpp::init(argc, argv);

    // Create the ImageSubscriber node
    auto image_subscriber_node = std::make_shared<ImageSubscriber>();

    // Call a method from the ImageSubscriber class
    image_subscriber_node->generate_hdr_photo();

    // Run the ImageSubscriber node
    rclcpp::spin(image_subscriber_node);

    // Clean up and shut down ROS 2
    rclcpp::shutdown();
    return 0;
}