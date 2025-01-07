//
// Created by Tim Mascal on 1/7/25.
//

#include "rclcpp/rclcpp.hpp"
#include "camera_tools/srv/change_exposure.hpp"

#include <memory>

void changeExposure(const std::shared_ptr<camera_tools::srv::ChangeExposure::Request> request,
          std::shared_ptr<camera_tools::srv::ChangeExposure::Response> response)
{

    // response->success = request->exposure_value;
    response->success = true;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                  request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("change_exposure_server");

    rclcpp::Service<camera_tools::srv::ChangeExposure>::SharedPtr service =
      node->create_service<camera_tools::srv::ChangeExposure>("change_exposure", &changeExposure);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to recieve command");

    rclcpp::spin(node);
    rclcpp::shutdown();
}
