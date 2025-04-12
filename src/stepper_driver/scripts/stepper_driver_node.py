#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# Set DEBUG_MODE if --debug flag is provided
DEBUG_MODE = "--debug" in sys.argv

if DEBUG_MODE:
    print("Debug mode enabled: using dummy GPIO.")
    class DummyGPIO:
        BCM = "BCM"
        OUT = "OUT"
        IN = "IN"
        HIGH = 1
        LOW = 0
        PUD_UP = "PUD_UP"
        def setmode(self, mode):
            print(f"DEBUG: set GPIO mode to {mode}")
        def setup(self, pin, mode, pull_up_down=None):
            print(f"DEBUG: setup pin {pin} as {mode} with pull_up_down={pull_up_down}")
        def output(self, pin, state):
            print(f"DEBUG: set pin {pin} to {state}")
        def input(self, pin):
            print(f"DEBUG: read pin {pin}")
            # Simulate payload done (for payload_done_pin) and endstop triggered (for endstop_pin) immediately.
            return self.HIGH
        def cleanup(self):
            print("DEBUG: cleanup GPIO")
    GPIO = DummyGPIO()
else:
    import RPi.GPIO as GPIO

class PayloadDeployer(Node):
    def __init__(self):
        super().__init__('payload_deployer')
        # Create the deploy service using the standard Trigger service.
        self.srv = self.create_service(Trigger, 'deploy_payload', self.deploy_callback)

        # Define GPIO pins (BCM numbering)
        self.STEP_PIN = 18       # Pin that sends step pulses to the A4988
        self.DIR_PIN = 23        # Pin that sets motor direction (HIGH for extend, LOW for retract)
        self.ENABLE_PIN = 24     # Pin to enable/disable the driver (LOW = enabled)
        self.ENDSTOP_PIN = 25    # Input pin for the endstop switch
        self.PAYLOAD_DONE_PIN = 12  # Input pin for payload done signal

        # Initialize GPIO library and pin modes.
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        GPIO.setup(self.ENABLE_PIN, GPIO.OUT)
        GPIO.setup(self.ENDSTOP_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.PAYLOAD_DONE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Enable the stepper driver (A4988 is enabled when ENABLE_PIN is LOW)
        GPIO.output(self.ENABLE_PIN, GPIO.LOW)

        # Movement parameters
        self.step_delay = 0.001      # Delay between steps (in seconds) - adjust for desired speed.
        self.steps_forward = 1000    # Number of steps to fully extend. Tune to your hardware.

    def step_motor(self, steps):
        """Pulse the step pin a given number of times to move the motor."""
        if DEBUG_MODE:
            for i in range(steps):
                print(f"DEBUG: Pulsing STEP_PIN (simulation), step {i+1}/{steps}")
                time.sleep(self.step_delay)
        else:
            for i in range(steps):
                GPIO.output(self.STEP_PIN, GPIO.HIGH)
                time.sleep(self.step_delay)
                GPIO.output(self.STEP_PIN, GPIO.LOW)
                time.sleep(self.step_delay)

    def deploy_callback(self, request, response):
        """
        Service callback triggered by a deployment request.
        Deployment phases:
          1. Extend the lead screw fully.
          2. Wait for payload action to complete.
          3. Retract the lead screw until the endstop is triggered.
        """
        try:
            # 1. Extend lead screw
            self.get_logger().info("Deploy request received. Extending lead screw...")
            GPIO.output(self.DIR_PIN, GPIO.HIGH)  # Set direction for extension.
            self.step_motor(self.steps_forward)

            # 2. Wait for payload to complete its action.
            self.get_logger().info("Waiting for payload operation to complete...")
            if DEBUG_MODE:
                self.get_logger().info("DEBUG: Simulating payload operation complete.")
                time.sleep(1)  # Simulate a short delay.
            else:
                while GPIO.input(self.PAYLOAD_DONE_PIN) == GPIO.LOW:
                    time.sleep(0.1)
                self.get_logger().info("Payload operation complete.")

            # 3. Retract lead screw until the endstop is triggered.
            self.get_logger().info("Retracting lead screw until endstop is triggered...")
            GPIO.output(self.DIR_PIN, GPIO.LOW)  # Set direction for retraction.
            if DEBUG_MODE:
                self.get_logger().info("DEBUG: Simulating endstop triggered immediately.")
                time.sleep(1)
            else:
                # Assuming the endstop switch pulls the pin LOW when triggered.
                while GPIO.input(self.ENDSTOP_PIN) == GPIO.HIGH:
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)
                    time.sleep(self.step_delay)
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                    time.sleep(self.step_delay)
                self.get_logger().info("Endstop triggered. Retracted successfully.")

            response.success = True
            response.message = "Payload deployed and retracted successfully."
        except Exception as e:
            self.get_logger().error("Error during deployment: " + str(e))
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        """Cleanup GPIO before shutting down the node."""
        GPIO.cleanup()
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