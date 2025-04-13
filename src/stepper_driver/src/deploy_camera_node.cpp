#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <gpiod.h>
#include <signal.h>
#include <time.h>
#include <thread>
#include <atomic>

#define CHIPNAME "/dev/gpiochip0"
#define STEP_PIN 18  // Step pin (BCM numbering)
#define DIR_PIN  17  // Direction pin (BCM numbering)

// Atomic flags to avoid race conditions
std::atomic<bool> deploying(false);
std::atomic<bool> running(false);

void deploy_camera_logic() {
    int ret;
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
    RCLCPP_INFO(rclcpp::get_logger("deploy_camera_service"), "Starting step signal generation on GPIO %d", STEP_PIN);
    // Set up signal handler (if needed) for graceful shutdown
    // Note: ROS2 already handles SIGINT; the following loop will stop on node shutdown.
    running = true;
    // Define a timespec for a 500 microsecond delay
    struct timespec delay = {0, 500000}; // 500,000 ns = 500 µs
    while (running && rclcpp::ok()) {
        // Set step line HIGH, delay, then LOW, and delay again
        gpiod_line_set_value(step_line, 1);
        nanosleep(&delay, NULL);
        gpiod_line_set_value(step_line, 0);
        nanosleep(&delay, NULL);
    }
    RCLCPP_INFO(rclcpp::get_logger("deploy_camera_service"), "Stopping step signal generation.");
    // Clean up resources
    gpiod_line_release(step_line);
    gpiod_line_release(dir_line);
    gpiod_chip_close(chip);
    deploying = false;
}

class DeployCameraService : public rclcpp::Node {
public:
    DeployCameraService() : Node("deploy_camera_service") {
        service_ = this->create_service<std_srvs::srv::Trigger>("deploy_camera", std::bind(&DeployCameraService::handle_deploy_camera, this, std::placeholders::_1, std::placeholders::_2));
    }
    
private:
    void handle_deploy_camera(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        if (!deploying) {
            deploying = true;
            // Launch the deploy logic in a background thread
            std::thread(deploy_camera_logic).detach();
            response->success = true;
            response->message = "Camera deployment started.";
        } else {
            response->success = false;
            response->message = "Camera deployment is already in progress.";
        }
    }
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
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