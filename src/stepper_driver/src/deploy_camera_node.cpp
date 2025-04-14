#include <rclcpp/rclcpp.hpp>
#include <gpiod.h>
#include <signal.h>
#include <time.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <string>
#include "stepper_driver/srv/deploy_camera_distance.hpp"  // Custom service header
#include "std_srvs/srv/trigger.hpp"                      // Using Trigger for retract_camera service

#define CHIPNAME "/dev/gpiochip0"
#define STEP_PIN 18       // Step pin (BCM numbering)
#define DIR_PIN  17       // Direction pin (BCM numbering)
#define ENDSTOP_PIN 27    // Endstop pin (BCM numbering)
#define STEPS_PER_MM 25   // 200 steps/rev divided by 8 mm pitch

// Atomic flags to avoid race conditions.
std::atomic<bool> deploying(false);
std::atomic<bool> running(false);

//
// deploy_camera_logic: Deploys the camera by generating a fixed number of step pulses.
// It requests two output lines (the step line and the direction line) using the modern request API.
//
void deploy_camera_logic(double distance) {
    int ret;
    bool reverse = false;
    if (distance < 0) {
        reverse = true;
        distance = -distance;
    }
    int total_steps = static_cast<int>(distance * STEPS_PER_MM);
    int step_count = 0;

    // Open the GPIO chip.
    struct gpiod_chip *chip = gpiod_chip_open(CHIPNAME);
    if (!chip) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to open chip");
        deploying = false;
        return;
    }

    // Create and configure the request configuration.
    struct gpiod_request_config *req_cfg = gpiod_request_config_new();
    if (!req_cfg) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to create request config");
        gpiod_chip_close(chip);
        deploying = false;
        return;
    }
    gpiod_request_config_set_consumer(req_cfg, "deploy_camera");

    // Create and set up the line configuration.
    struct gpiod_line_config *line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to create line config");
        gpiod_request_config_free(req_cfg);
        gpiod_chip_close(chip);
        deploying = false;
        return;
    }

    // Configure the step line (output, initial value = inactive).
    unsigned int step_offsets[1] = { STEP_PIN };
    struct gpiod_line_settings *settings_step = gpiod_line_settings_new();
    if (!settings_step) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to create settings for step line");
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        gpiod_chip_close(chip);
        deploying = false;
        return;
    }
    ret = gpiod_line_settings_set_direction(settings_step, GPIOD_LINE_DIRECTION_OUTPUT);
    ret |= gpiod_line_settings_set_output_value(settings_step, GPIOD_LINE_VALUE_INACTIVE);
    ret |= gpiod_line_config_add_line_settings(line_cfg, step_offsets, 1, settings_step);
    gpiod_line_settings_free(settings_step);
    if (ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to set settings for step line");
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        gpiod_chip_close(chip);
        deploying = false;
        return;
    }

    // Configure the direction line (output, initial value depends on direction).
    unsigned int dir_offsets[1] = { DIR_PIN };
    struct gpiod_line_settings *settings_dir = gpiod_line_settings_new();
    if (!settings_dir) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to create settings for direction line");
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        gpiod_chip_close(chip);
        deploying = false;
        return;
    }
    ret = gpiod_line_settings_set_direction(settings_dir, GPIOD_LINE_DIRECTION_OUTPUT);
    ret |= gpiod_line_settings_set_output_value(settings_dir,
           reverse ? GPIOD_LINE_VALUE_INACTIVE : GPIOD_LINE_VALUE_ACTIVE);
    ret |= gpiod_line_config_add_line_settings(line_cfg, dir_offsets, 1, settings_dir);
    gpiod_line_settings_free(settings_dir);
    if (ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to set settings for direction line");
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        gpiod_chip_close(chip);
        deploying = false;
        return;
    }

    // Request the two lines.
    struct gpiod_line_request *request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    if (!request) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to request lines for deployment");
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        gpiod_chip_close(chip);
        deploying = false;
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("deploy_camera_service"),
                "Starting step generation for %.2f mm (%s, total steps: %d)",
                distance, reverse ? "reverse" : "forward", total_steps);

    running = true;
    struct timespec delay = {0, 500000}; // 500 µs delay

    // Toggle the step line until all steps are complete.
    while (running && rclcpp::ok() && step_count < total_steps) {
        ret = gpiod_line_request_set_value(request, STEP_PIN, GPIOD_LINE_VALUE_ACTIVE);
        if (ret < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to set step line high");
            break;
        }
        nanosleep(&delay, nullptr);
        ret = gpiod_line_request_set_value(request, STEP_PIN, GPIOD_LINE_VALUE_INACTIVE);
        if (ret < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to set step line low");
            break;
        }
        nanosleep(&delay, nullptr);
        step_count++;
    }
    RCLCPP_INFO(rclcpp::get_logger("deploy_camera_service"),
                "Completed stepping: %d/%d steps.", step_count, total_steps);

    // Clean up.
    gpiod_line_request_release(request);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);
    gpiod_chip_close(chip);
    deploying = false;
}

//
// return_camera_logic: Retracts the camera until the endstop line reads active.
// It requests three lines: step (output), direction (output), and endstop (input with pull‑up).
//
void return_camera_logic() {
    int ret;
    struct gpiod_chip *chip = gpiod_chip_open(CHIPNAME);
    if (!chip) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to open chip for return motion");
        return;
    }

    struct gpiod_request_config *req_cfg = gpiod_request_config_new();
    if (!req_cfg) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to create request config");
        gpiod_chip_close(chip);
        return;
    }
    gpiod_request_config_set_consumer(req_cfg, "return_camera");

    struct gpiod_line_config *line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to create line config");
        gpiod_request_config_free(req_cfg);
        gpiod_chip_close(chip);
        return;
    }

    // Configure the step line (output, inactive).
    unsigned int step_offsets[1] = { STEP_PIN };
    struct gpiod_line_settings *settings_step = gpiod_line_settings_new();
    ret = gpiod_line_settings_set_direction(settings_step, GPIOD_LINE_DIRECTION_OUTPUT);
    ret |= gpiod_line_settings_set_output_value(settings_step, GPIOD_LINE_VALUE_INACTIVE);
    ret |= gpiod_line_config_add_line_settings(line_cfg, step_offsets, 1, settings_step);
    gpiod_line_settings_free(settings_step);
    if (ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to configure step line for return motion");
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        gpiod_chip_close(chip);
        return;
    }

    // Configure the direction line (output, set to inactive for reverse).
    unsigned int dir_offsets[1] = { DIR_PIN };
    struct gpiod_line_settings *settings_dir = gpiod_line_settings_new();
    ret = gpiod_line_settings_set_direction(settings_dir, GPIOD_LINE_DIRECTION_OUTPUT);
    ret |= gpiod_line_settings_set_output_value(settings_dir, GPIOD_LINE_VALUE_INACTIVE);
    ret |= gpiod_line_config_add_line_settings(line_cfg, dir_offsets, 1, settings_dir);
    gpiod_line_settings_free(settings_dir);
    if (ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to configure direction line for return motion");
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        gpiod_chip_close(chip);
        return;
    }

    // Configure the endstop line (input, with pull-up bias).
    unsigned int endstop_offsets[1] = { ENDSTOP_PIN };
    struct gpiod_line_settings *settings_endstop = gpiod_line_settings_new();
    ret = gpiod_line_settings_set_direction(settings_endstop, GPIOD_LINE_DIRECTION_INPUT);
    ret |= gpiod_line_settings_set_bias(settings_endstop, GPIOD_LINE_BIAS_PULL_UP);
    ret |= gpiod_line_config_add_line_settings(line_cfg, endstop_offsets, 1, settings_endstop);
    gpiod_line_settings_free(settings_endstop);
    if (ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to configure endstop line for return motion");
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        gpiod_chip_close(chip);
        return;
    }

    // Request all three lines.
    struct gpiod_line_request *request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    if (!request) {
        RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to request lines for return motion");
        gpiod_line_config_free(line_cfg);
        gpiod_request_config_free(req_cfg);
        gpiod_chip_close(chip);
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("deploy_camera_service"),
                "Starting return motion until endstop (pin %d) is triggered.", ENDSTOP_PIN);

    struct timespec delay = {0, 500000}; // 500 µs delay

    while (rclcpp::ok()) {
        int endstop_value = gpiod_line_request_get_value(request, ENDSTOP_PIN);
        if (endstop_value < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Error reading endstop value");
            break;
        }
        if (endstop_value == GPIOD_LINE_VALUE_ACTIVE) {
            RCLCPP_INFO(rclcpp::get_logger("deploy_camera_service"), "Endstop triggered. Stopping return motion.");
            break;
        }
        // Generate a step pulse.
        ret = gpiod_line_request_set_value(request, STEP_PIN, GPIOD_LINE_VALUE_ACTIVE);
        if (ret < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to set step line high during return motion");
            break;
        }
        nanosleep(&delay, nullptr);
        ret = gpiod_line_request_set_value(request, STEP_PIN, GPIOD_LINE_VALUE_INACTIVE);
        if (ret < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("deploy_camera_service"), "Failed to set step line low during return motion");
            break;
        }
        nanosleep(&delay, nullptr);
    }

    // Clean up.
    gpiod_line_request_release(request);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);
    gpiod_chip_close(chip);
}

//
// DeployCameraService: A ROS2 node providing two services for deploying and retracting the camera.
//
class DeployCameraService : public rclcpp::Node {
public:
    DeployCameraService() : Node("deploy_camera_service") {
        deploy_service_ = this->create_service<stepper_driver::srv::DeployCameraDistance>(
            "deploy_camera",
            std::bind(&DeployCameraService::handle_deploy_camera, this,
                      std::placeholders::_1, std::placeholders::_2)
        );
        retract_service_ = this->create_service<std_srvs::srv::Trigger>(
            "retract_camera",
            std::bind(&DeployCameraService::handle_retract_camera, this,
                      std::placeholders::_1, std::placeholders::_2)
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
        RCLCPP_INFO(this->get_logger(), "Initiating camera retraction sequence...");
        std::thread(return_camera_logic).detach();
        response->success = true;
        response->message = "Camera retraction initiated. Motor returning until endstop is triggered.";
    }
    
    rclcpp::Service<stepper_driver::srv::DeployCameraDistance>::SharedPtr deploy_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr retract_service_;
};

//
// main: Initializes the ROS2 node and spins until shutdown. Upon shutdown, it signals any running deploy motion to stop.
//
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeployCameraService>();
    rclcpp::spin(node);
    running = false;
    rclcpp::shutdown();
    return 0;
}
