#include <rclcpp/rclcpp.hpp>
#include <gpiod.h>
#include <signal.h>
#include <time.h>
#include <thread>
#include <atomic>
#include <chrono>
#include "stepper_driver/srv/deploy_camera_distance.hpp"  // Custom service header
#include "std_srvs/srv/trigger.hpp"                      // Using Trigger for retract_camera service

#define CHIPNAME "/dev/gpiochip0"
#define STEP_PIN 18  // Step pin (BCM numbering)
#define DIR_PIN  17  // Direction pin (BCM numbering)
#define ENDSTOP_PIN 27  // Endstop pin (BCM numbering)
#define STEPS_PER_MM 25  // 200 steps/rev divided by 8 mm pitch

// Atomic flags to avoid race conditions
std::atomic<bool> deploying(false);
std::atomic<bool> running(false);

void deploy_camera_logic(double distance) {
    int ret;
    bool reverse = false;
    if (distance < 0) {
        reverse = true;
        distance = -distance;  // Use absolute distance for steps calculation
    }
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
    // Request step line as output
    ret = gpiod_line_request_output(step_line, "deploy_camera", 0);
    if (ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to request step line as output");
        gpiod_chip_close(chip);
        deploying = false;
        return;
    }
    // Request direction line as output with initial value based on whether rolling in reverse or forward
    ret = gpiod_line_request_output(dir_line, "deploy_camera", reverse ? 0 : 1);
    if (ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to request direction line as output");
        gpiod_line_release(step_line);
        gpiod_chip_close(chip);
        deploying = false;
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("deploy_camera_service"), "Starting step signal generation for %.2f mm (%s, total steps: %d)", 
                 distance, (reverse ? "reverse" : "forward"), total_steps);
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

void return_camera_logic() {
    int ret;
    // Open the GPIO chip
    struct gpiod_chip* chip = gpiod_chip_open(CHIPNAME);
    if (!chip) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to open chip for return motion");
        return;
    }
    // Get lines for step, direction, and endstop
    struct gpiod_line* step_line = gpiod_chip_get_line(chip, STEP_PIN);
    struct gpiod_line* dir_line = gpiod_chip_get_line(chip, DIR_PIN);
    struct gpiod_line* endstop_line = gpiod_chip_get_line(chip, ENDSTOP_PIN);

    // Request endstop line as input with pull-up mode
    struct gpiod_line_request_config endstop_config = {.consumer = "return_camera",.direction = GPIOD_LINE_REQUEST_DIRECTION_INPUT,.flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP};
    ret = gpiod_line_request(endstop_line, &endstop_config, 0);
    if(ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to request endstop line as input with pull-up bias");
        gpiod_line_release(step_line);
        gpiod_line_release(dir_line);
        gpiod_chip_close(chip);
        return;
    }
    if (!step_line || !dir_line || !endstop_line) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to get one or more lines for return motion");
        if(chip) gpiod_chip_close(chip);
        return;
    }
    // Request step and direction lines as outputs
    ret = gpiod_line_request_output(step_line, "return_camera", 0);
    if(ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to request step line for return motion");
        gpiod_chip_close(chip);
        return;
    }
    // Set the direction for reverse motion. (Assuming 0 corresponds to reverse)
    ret = gpiod_line_set_value(dir_line, 0);
    if(ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to set direction for return motion");
        gpiod_line_release(step_line);
        gpiod_chip_close(chip);
        return;
    }
    // Request endstop line as input
    ret = gpiod_line_request_input(endstop_line, "return_camera");
    if(ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to request endstop line as input");
        gpiod_line_release(step_line);
        gpiod_line_release(dir_line);
        gpiod_chip_close(chip);
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("deploy_camera_service"), "Starting return motion until endstop (pin %d) is triggered.", ENDSTOP_PIN);
    struct timespec delay = {0, 500000}; // 500 µs delay
    while(rclcpp::ok()) {
        // Read the endstop state
        int endstop_value = gpiod_line_get_value(endstop_line);
        if(endstop_value < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Error reading endstop value");
            break;
        }
        // Assuming active high means the endstop is triggered
        if(endstop_value == 1) {
            RCLCPP_INFO(rclcpp::get_logger("deploy_camera_service"), "Endstop triggered. Stopping return motion.");
            break;
        }
        // Generate one step pulse
        gpiod_line_set_value(step_line, 1);
        nanosleep(&delay, nullptr);
        gpiod_line_set_value(step_line, 0);
        nanosleep(&delay, nullptr);
    }
    // Clean up resources
    gpiod_line_release(step_line);
    gpiod_line_release(dir_line);
    gpiod_line_release(endstop_line);
    gpiod_chip_close(chip);
}

class DeployCameraService : public rclcpp::Node {
public:
    DeployCameraService() : Node("deploy_camera_service") {
        // Service to start camera deployment with a given distance
        deploy_service_ = this->create_service<stepper_driver::srv::DeployCameraDistance>(
            "deploy_camera",
            std::bind(&DeployCameraService::handle_deploy_camera, this, std::placeholders::_1, std::placeholders::_2)
        );
        // Service to retract the camera (previously "complete_payload")
        retract_service_ = this->create_service<std_srvs::srv::Trigger>(
            "retract_camera",
            std::bind(&DeployCameraService::handle_retract_camera, this, std::placeholders::_1, std::placeholders::_2)
        );
    }
    
private:
    void handle_deploy_camera(
        const std::shared_ptr<stepper_driver::srv::DeployCameraDistance::Request> request,
        std::shared_ptr<stepper_driver::srv::DeployCameraDistance::Response> response) 
    {
        if (!deploying) {
            deploying = true;
            std::thread(deploy_camera_logic, request->distance).detach();
            response->success = true;
            response->message = "Camera deployment started for distance: " + std::to_string(request->distance) + " mm.";
        } else {
            response->success = false;
            response->message = "Camera deployment is already in progress.";
        }
    }
    
    void handle_retract_camera(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        // Optionally include any pre-retraction tasks here.
        RCLCPP_INFO(this->get_logger(), "Initiating camera retraction sequence...");
        
        // Launch return (retraction) motion in a background thread.
        std::thread(return_camera_logic).detach();

        response->success = true;
        response->message = "Camera retraction initiated. Motor returning until endstop is triggered.";
    }
    
    rclcpp::Service<stepper_driver::srv::DeployCameraDistance>::SharedPtr deploy_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr retract_service_;
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