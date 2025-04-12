#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

DEBUG_MODE = "--debug" in sys.argv

if DEBUG_MODE:
    print("Debug mode enabled: using dummy devices.")

    class DummyDevice:
        def __init__(self, pin, **kwargs):
            self.pin = pin
            self.state = False
            print(f"DEBUG: Initialized dummy device at pin {pin} with {kwargs}")

        def on(self):
            self.state = True
            print(f"DEBUG: Device at pin {self.pin} set ON")

        def off(self):
            self.state = False
            print(f"DEBUG: Device at pin {self.pin} set OFF")

        @property
        def value(self):
            print(f"DEBUG: Reading dummy device at pin {self.pin}")
            # Always simulate HIGH for inputs
            return 1

    DigitalOutputDevice = DummyDevice
    DigitalInputDevice = DummyDevice

else:
    from gpiozero import DigitalOutputDevice, DigitalInputDevice

class PayloadDeployer(Node):
    def __init__(self):
        super().__init__('payload_deployer')
        # Create the deploy service.
        self.srv = self.create_service(Trigger, 'deploy_payload', self.deploy_callback)

        # Define pins using BCM numbering.
        self.STEP_PIN = 18        # Step pulse pin.
        self.DIR_PIN = 23         # Motor direction.
        self.ENABLE_PIN = 24      # Motor enable (LOW = enabled).
        self.ENDSTOP_PIN = 25     # Endstop input.
        self.PAYLOAD_DONE_PIN = 12  # Payload done input.

        # Setup output devices.
        # For the ENABLE output, active_high is False because LOW = enabled.
        self.step_output = DigitalOutputDevice(self.STEP_PIN)
        self.direction = DigitalOutputDevice(self.DIR_PIN)
        self.enable = DigitalOutputDevice(self.ENABLE_PIN, active_high=False)
        # Setup input devices with pull-ups.
        self.endstop = DigitalInputDevice(self.ENDSTOP_PIN, pull_up=True)
        self.payload_done = DigitalInputDevice(self.PAYLOAD_DONE_PIN, pull_up=True)

        # Enable the stepper driver.
        self.enable.on()

        # Movement parameters.
        self.step_delay = 0.15    # Delay for each half-step pulse.
        self.steps_forward = 200  # One full revolution equals 200 steps.

    def step_motor(self, steps):
        """Pulse the step pin to move the motor a specified number of steps."""
        for i in range(steps):
            if DEBUG_MODE:
                self.get_logger().info(f"DEBUG: Would pulse STEP_PIN, step {i+1}/{steps}")
                time.sleep(self.step_delay)
            else:
                self.step_output.on()
                time.sleep(self.step_delay)
                self.step_output.off()
                time.sleep(self.step_delay)

    def deploy_callback(self, request, response):
        """
        Deployment sequence:
          1. Extend the lead screw.
          2. Wait for payload operation to complete.
          3. Retract until the endstop is triggered.
        """
        try:
            # 1. Extend lead screw.
            self.get_logger().info("Deploy request received. Extending lead screw...")
            self.direction.on()  # Set direction for extension.
            self.step_motor(self.steps_forward)

            # 2. Wait for payload completion.
            self.get_logger().info("Waiting for payload operation to complete...")
            if DEBUG_MODE:
                self.get_logger().info("DEBUG: Simulating payload completion.")
                time.sleep(1)
            else:
                # Wait until payload_done becomes HIGH.
                while self.payload_done.value == 0:
                    time.sleep(0.1)
                self.get_logger().info("Payload operation complete.")

            # 3. Retract lead screw until endstop triggered.
            self.get_logger().info("Retracting lead screw until endstop is triggered...")
            self.direction.off()  # Set direction for retraction.
            if DEBUG_MODE:
                self.get_logger().info("DEBUG: Simulating immediate endstop trigger.")
                time.sleep(1)
            else:
                # Assuming endstop goes LOW when triggered.
                while self.endstop.value == 1:
                    self.step_output.on()
                    time.sleep(self.step_delay)
                    self.step_output.off()
                    time.sleep(self.step_delay)
                self.get_logger().info("Endstop triggered. Retraction complete.")

            response.success = True
            response.message = "Payload deployed and retracted successfully."
        except Exception as e:
            self.get_logger().error("Error during deployment: " + str(e))
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        self.get_logger().info("Cleaning up devices.")
        # gpiozero devices require no explicit cleanup.
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PayloadDeployer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()