import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from gpiozero import DigitalOutputDevice, Device
from gpiozero.pins.mock import MockFactory
import os
import time
import threading

class UnipolarStepperNode(Node):
    """
    ROS2 Node for controlling a unipolar stepper motor via ULN2003 driver.
    Supports step control and continuous rotation at a specified speed.
    """

    def __init__(self):
        super().__init__('unipolar_stepper_node')

        # 1. Declare Parameters
        self.declare_parameter('in1_pin', 17) # GPIO BCM
        self.declare_parameter('in2_pin', 27) # GPIO BCM
        self.declare_parameter('in3_pin', 22) # GPIO BCM
        self.declare_parameter('in4_pin', 23) # GPIO BCM
        self.declare_parameter('steps_per_revolution', 2048) # For 28BYJ-48 motor (5.625 degrees / step * 64 reduction ratio)
        self.declare_parameter('mock_hardware', False)
        self.declare_parameter('step_delay_ms', 2) # Milliseconds delay between steps for speed control

        # 2. Read Parameters
        self.in_pins = [
            self.get_parameter('in1_pin').value,
            self.get_parameter('in2_pin').value,
            self.get_parameter('in3_pin').value,
            self.get_parameter('in4_pin').value
        ]
        self.steps_per_revolution = self.get_parameter('steps_per_revolution').value
        self.mock_mode = self.get_parameter('mock_hardware').value
        self.step_delay_ms = self.get_parameter('step_delay_ms').value / 1000.0 # Convert to seconds

        # Full step sequence for 4-phase unipolar motor
        self.step_sequence = [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]
        self.step_index = 0

        # Initialize mock factory if needed
        if self.mock_mode or os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock':
            self.get_logger().warn('Stepper: Running in MOCK mode. No real GPIO will be used.')
            Device.pin_factory = MockFactory()

        # 3. Hardware Init
        self.motor_pins = []
        try:
            for pin in self.in_pins:
                self.motor_pins.append(DigitalOutputDevice(pin, initial_value=False))
            self.get_logger().info(f'Stepper: Initialized ULN2003 on GPIOs {self.in_pins}.')
        except Exception as e:
            self.get_logger().error(f'Stepper: Failed to initialize GPIO for ULN2003: {e}. Falling back to mock.')
            self.mock_mode = True # Fallback to mock if real GPIO fails
            # Create dummy devices for mock mode if not already done
            for pin in self.in_pins:
                self.motor_pins.append(MockDigitalOutputDevice(pin))
            
        self._stop_motor() # Ensure motor is off initially

        # 4. Control variables
        self.target_steps = 0
        self.current_speed = 0.0 # -1.0 to 1.0
        self.control_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.motor_thread = None

        # 5. Subscribers
        self.steps_sub = self.create_subscription(
            Int32,
            '~/cmd_steps',
            self.cmd_steps_callback,
            10
        )
        self.speed_sub = self.create_subscription(
            Float32,
            '~/cmd_speed',
            self.cmd_speed_callback,
            10
        )
        self.get_logger().info('Stepper: Subscribing to step and speed commands.')

    def _set_motor_pins(self, step):
        """Sets the state of the motor driver pins according to the step sequence."""
        for i in range(4):
            if self.mock_mode:
                if self.step_sequence[step][i] == 1:
                    self.motor_pins[i].on()
                else:
                    self.motor_pins[i].off()
            else:
                if self.step_sequence[step][i] == 1:
                    self.motor_pins[i].on()
                else:
                    self.motor_pins[i].off()

    def _stop_motor(self):
        """Turns off all motor driver pins."""
        if self.motor_pins:
            for pin in self.motor_pins:
                pin.off()
            self.get_logger().debug("Stepper: Motor stopped, all pins off.")

    def _step(self, direction):
        """Performs a single step in the given direction."""
        if direction > 0: # Clockwise
            self.step_index = (self.step_index + 1) % 4
        else: # Counter-clockwise
            self.step_index = (self.step_index - 1 + 4) % 4
        self._set_motor_pins(self.step_index)
        time.sleep(self.step_delay_ms) # Controls max speed

    def _motor_control_thread(self):
        """Thread for executing steps based on target_steps or current_speed."""
        self.get_logger().info("Stepper: Motor control thread started.")
        while not self.stop_event.is_set():
            with self.control_lock:
                if self.target_steps != 0:
                    direction = 1 if self.target_steps > 0 else -1
                    self._step(direction)
                    self.target_steps -= direction
                elif self.current_speed != 0.0:
                    direction = 1 if self.current_speed > 0 else -1
                    self._step(direction)
                    # Adjust delay based on speed
                    time.sleep(self.step_delay_ms / abs(self.current_speed))
                else:
                    self._stop_motor() # Stop when no commands
                    # Use a small sleep to avoid busy-waiting when idle
                    time.sleep(0.01)
        self.get_logger().info("Stepper: Motor control thread stopped.")


    def cmd_steps_callback(self, msg: Int32):
        """Moves the motor by a specified number of steps."""
        with self.control_lock:
            self.target_steps += msg.data
            self.current_speed = 0.0 # Stop continuous motion
        self.get_logger().info(f'Stepper: Received steps command: {msg.data}. Target steps: {self.target_steps}')
        if not self.motor_thread or not self.motor_thread.is_alive():
            self.motor_thread = threading.Thread(target=self._motor_control_thread)
            self.motor_thread.start()

    def cmd_speed_callback(self, msg: Float32):
        """Sets the motor speed for continuous rotation."""
        speed = max(-1.0, min(1.0, msg.data)) # Clamp speed
        with self.control_lock:
            self.current_speed = speed
            self.target_steps = 0 # Stop step motion
        self.get_logger().info(f'Stepper: Received speed command: {self.current_speed}')
        if not self.motor_thread or not self.motor_thread.is_alive():
            self.motor_thread = threading.Thread(target=self._motor_control_thread)
            self.motor_thread.start()

    def destroy_node(self):
        self.stop_event.set()
        if self.motor_thread and self.motor_thread.is_alive():
            self.motor_thread.join() # Wait for thread to finish
        self._stop_motor() # Ensure motor is off
        for pin_device in self.motor_pins:
            pin_device.close() # Release GPIOs
        self.get_logger().info('Stepper: GPIOs released.')
        super().destroy_node()

# Simple MockDigitalOutputDevice for when GPIO initialization fails in mock mode
class MockDigitalOutputDevice:
    def __init__(self, pin):
        self.pin = pin
        self.value = False
        # logging.getLogger(__name__).debug(f'MockDigitalOutputDevice {pin} initialized')
    def on(self):
        self.value = True
        # logging.getLogger(__name__).debug(f'MockDigitalOutputDevice {self.pin} ON')
    def off(self):
        self.value = False
        # logging.getLogger(__name__).debug(f'MockDigitalOutputDevice {self.pin} OFF')
    def close(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = UnipolarStepperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
