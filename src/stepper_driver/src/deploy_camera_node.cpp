#include <rclcpp/rclcpp.hpp>
#include <gpiod.h>
#include <signal.h>
#include <time.h>
#include <thread>
#include <atomic>
#include "stepper_driver/srv/deploy_camera_distance.hpp"  // Custom service header

#define CHIPNAME "/dev/gpiochip0"
#define STEP_PIN 18  // Step pin (BCM numbering)
#define DIR_PIN  17  // Direction pin (BCM numbering)
#define STEPS_PER_MM 25  // 200 steps/rev divided by 8 mm pitch

// Atomic flags to avoid race conditions
std::atomic<bool> deploying(false);
std::atomic<bool> running(false);

void deploy_camera_logic(double distance) {
    int ret;
    // Calculate total number of steps for the given distance (in mm)
    int total_steps = static_cast<int>(distance * STEPS_PER_MM);
    int step_count = 0;
    
    // Open the GPIO chip
    struct gpiod_chip* chip = gpiod_chip_open(CHIPNAME);
    if (!chip) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to open chip");
        deploying = false;
        return;
    }
    // Get lines for step and direction
    struct gpiod_line* step_line = gpiod_chip_get_line(chip, STEP_PIN);
    struct gpiod_line* dir_line = gpiod_chip_get_line(chip, DIR_PIN);
    if (!step_line || !dir_line) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to get line");
        gpiod_chip_close(chip);
        deploying = false;
        return;
    }
    // Request lines as outputs
    ret = gpiod_line_request_output(step_line, "deploy_camera", 0);
    if (ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to request step line as output");
        gpiod_chip_close(chip);
        deploying = false;
        return;
    }
    ret = gpiod_line_request_output(dir_line, "deploy_camera", 1);
    if (ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to request direction line as output");
        gpiod_line_release(step_line);
        gpiod_chip_close(chip);
        deploying = false;
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("deploy_camera_service"), "Starting step signal generation for %.2f mm (total steps: %d)", distance, total_steps);
    running = true;
    struct timespec delay = {0, 500000}; // 500 µs delay

    while (running && rclcpp::ok() && step_count < total_steps) {
        // Generate one step pulse
        gpiod_line_set_value(step_line, 1);
        nanosleep(&delay, nullptr);
        gpiod_line_set_value(step_line, 0);
        nanosleep(&delay, nullptr);
        step_count++;
    }
    RCLCPP_INFO(rclcpp::get_logger("deploy_camera_service"), "Completed stepping: %d/%d steps.", step_count, total_steps);
    // Clean up resources
    gpiod_line_release(step_line);
    gpiod_line_release(dir_line);
    gpiod_chip_close(chip);
    deploying = false;
}

class DeployCameraService : public rclcpp::Node {
public:
    DeployCameraService() : Node("deploy_camera_service") {
        // Update service type from Trigger to our custom DeployCameraDistance service.
        service_ = this->create_service<stepper_driver::srv::DeployCameraDistance>(
            "deploy_camera",
            std::bind(&DeployCameraService::handle_deploy_camera, this, std::placeholders::_1, std::placeholders::_2)
        );
    }
    
private:
    void handle_deploy_camera(
        const std::shared_ptr<stepper_driver::srv::DeployCameraDistance::Request> request,
        std::shared_ptr<stepper_driver::srv::DeployCameraDistance::Response> response) 
    {
        if (!deploying) {
            deploying = true;
            // Launch the deploy logic in a background thread using the provided distance
            std::thread(deploy_camera_logic, request->distance).detach();
            response->success = true;
            response->message = "Camera deployment started for distance: " + std::to_string(request->distance) + " mm.";
        } else {
            response->success = false;
            response->message = "Camera deployment is already in progress.";
        }
    }
    
    rclcpp::Service<stepper_driver::srv::DeployCameraDistance>::SharedPtr service_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeployCameraService>();
    rclcpp::spin(node);
    // On shutdown, signal the deploy logic (if running) to stop
    running = false;
    rclcpp::shutdown();
    return 0;
}