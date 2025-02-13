//
// Created by Tim Mascal on 1/7/25.
//

#include "rclcpp/rclcpp.hpp"
#include "camera_tools_interfaces/srv/change_exposure.hpp"
#include <memory>
#include <sstream>

void changeExposure(const std::shared_ptr<camera_tools_interfaces::srv::ChangeExposure::Request> request,
          std::shared_ptr<camera_tools_interfaces::srv::ChangeExposure::Response> response)
{

    (void)(request); // This ignores 'request' but silences unused-variable warnings.

    // Inputs
    int new_exposure_value = request->exposure_value;
    response->success = false;

    // Video Device
    int video_device = 2;

    // Define Command Variable to Set Exposure Mode to Manual
    if (new_exposure_value == 0) {
        std::stringstream enable_command_stream;
        enable_command_stream << "v4l2-ctl -d /dev/video" << video_device << " -c auto_exposure=3" ; // index 1 sets exposure mode to manual
        std::string enable_command = enable_command_stream.str();

        // Execute Command and recored response
        int enable_returnCode = std::system(enable_command.c_str());
        RCLCPP_INFO(rclcpp::get_logger("change_exposure"), "Command executed with return code: %d", enable_returnCode);

        if (enable_returnCode == 0) {
            response->success = true;
            RCLCPP_INFO(rclcpp::get_logger("change_exposure"), "Command executed successfully, exposure set to: automatic");
        } else {
            response->success = false;
            RCLCPP_ERROR(rclcpp::get_logger("change_exposure"), "Command failed with return code: %d", enable_returnCode);
        }

    } else {
    std::stringstream enable_command_stream;
    enable_command_stream << "v4l2-ctl -d /dev/video" << video_device << " -c auto_exposure=1" ; // index 1 sets exposure mode to manual
    std::string enable_command = enable_command_stream.str();

    // Execute Command and recored response
    int enable_returnCode = std::system(enable_command.c_str());
    RCLCPP_INFO(rclcpp::get_logger("change_exposure"), "Command executed with return code: %d", enable_returnCode);

    // Define Command Variable
    std::stringstream command_stream;
    command_stream << "v4l2-ctl -d /dev/video" << video_device << " -c exposure_time_absolute=" << new_exposure_value;
    std::string command = command_stream.str();

    // Execute Command and recored response
    int returnCode = std::system(command.c_str());

    // Check if the command was executed successfully
    if (returnCode == 0) {
        response->success = true;
        RCLCPP_INFO(rclcpp::get_logger("change_exposure"), "Command executed successfully, exposure set to: %d", new_exposure_value);
    } else {
        response->success = false;
        RCLCPP_ERROR(rclcpp::get_logger("change_exposure"), "Command failed with return code: %d", returnCode);
    }
 }


    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld", request->a, request->b);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("change_exposure_server");

    rclcpp::Service<camera_tools_interfaces::srv::ChangeExposure>::SharedPtr service =
      node->create_service<camera_tools_interfaces::srv::ChangeExposure>("change_exposure", &changeExposure);

    RCLCPP_INFO(rclcpp::get_logger("change_exposure"), "Ready to recieve command");

    rclcpp::spin(node);
    rclcpp::shutdown();
}
