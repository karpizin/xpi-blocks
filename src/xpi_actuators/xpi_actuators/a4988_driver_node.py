import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from gpiozero import DigitalOutputDevice, Device
from gpiozero.pins.mock import MockFactory
import os
import time
import threading

class A4988DriverNode(Node):
    """
    ROS2 Node for controlling a bipolar stepper motor via A4988/DRV8825 driver.
    Uses STEP/DIR pins. Supports step control and continuous rotation at a specified speed.
    """

    def __init__(self):
        super().__init__('a4988_driver_node')

        # 1. Declare Parameters
        self.declare_parameter('step_pin', 17) # GPIO BCM
        self.declare_parameter('dir_pin', 27)  # GPIO BCM
        self.declare_parameter('enable_pin', None) # Optional, if used (active low)
        self.declare_parameter('microstep_pins', []) # e.g., [22, 23, 24] for MS1, MS2, MS3
        self.declare_parameter('microstep_setting', 1) # Full step (1) or 2, 4, 8, 16, 32...
        self.declare_parameter('steps_per_revolution_fullstep', 200) # For typical NEMA17 (1.8 deg/step)
        self.declare_parameter('mock_hardware', False)
        self.declare_parameter('step_delay_s', 0.001) # Minimum delay in seconds between steps (lower = faster)
        self.declare_parameter('motor_max_rpm', 60) # For calculating speed from -1.0 to 1.0

        # 2. Read Parameters
        self.step_pin = self.get_parameter('step_pin').value
        self.dir_pin = self.get_parameter('dir_pin').value
        self.enable_pin = self.get_parameter('enable_pin').value
        self.microstep_pins = self.get_parameter('microstep_pins').value
        self.microstep_setting = self.get_parameter('microstep_setting').value
        self.steps_per_revolution_fullstep = self.get_parameter('steps_per_revolution_fullstep').value
        self.mock_mode = self.get_parameter('mock_hardware').value
        self.step_delay_s = self.get_parameter('step_delay_s').value
        self.motor_max_rpm = self.get_parameter('motor_max_rpm').value

        self.current_microstep = self.microstep_setting
        self.effective_steps_per_revolution = self.steps_per_revolution_fullstep * self.current_microstep

        # Initialize mock factory if needed
        if self.mock_mode or os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock':
            self.get_logger().warn('A4988: Running in MOCK mode. No real GPIO will be used.')
            Device.pin_factory = MockFactory()

        # 3. Hardware Init
        self.step_device = None
        self.dir_device = None
        self.enable_device = None
        self.ms_devices = []

        try:
            self.step_device = DigitalOutputDevice(self.step_pin, initial_value=False)
            self.dir_device = DigitalOutputDevice(self.dir_pin, initial_value=False)
            self.get_logger().info(f'A4988: Initialized STEP on GPIO {self.step_pin}, DIR on GPIO {self.dir_pin}.')

            if self.enable_pin is not None:
                self.enable_device = DigitalOutputDevice(self.enable_pin, initial_value=True) # Active low
                self.get_logger().info(f'A4988: Initialized ENABLE on GPIO {self.enable_pin}.')

            for pin in self.microstep_pins:
                self.ms_devices.append(DigitalOutputDevice(pin, initial_value=False))
            self._set_microstepping(self.current_microstep)
            self.get_logger().info(f'A4988: Microstepping set to 1/{self.current_microstep}.')

            self.disable_motor() # Ensure motor is off initially if enable pin is used
        except Exception as e:
            self.get_logger().error(f'A4988: Failed to initialize GPIO: {e}. Falling back to mock.')
            self.mock_mode = True # Fallback to mock if real GPIO fails
            self.step_device = MockDigitalOutputDevice(self.step_pin)
            self.dir_device = MockDigitalOutputDevice(self.dir_pin)
            if self.enable_pin is not None:
                self.enable_device = MockDigitalOutputDevice(self.enable_pin)
            for pin in self.microstep_pins:
                self.ms_devices.append(MockDigitalOutputDevice(pin))

        # 4. Control variables
        self.target_steps = 0
        self.current_speed_factor = 0.0 # -1.0 to 1.0 (relative to max RPM)
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
        self.get_logger().info('A4988: Subscribing to step and speed commands.')

    def _set_microstepping(self, setting: int):
        """
        Sets the microstepping pins based on the desired setting.
        For A4988: MS1, MS2, MS3 pins correspond to 1, 2, 4, 8, 16.
        For DRV8825: MS0, MS1, MS2 pins correspond to 1, 2, 4, 8, 16, 32.
        This function assumes MS1, MS2, MS3 pins are in order in `self.microstep_pins`.
        """
        if not self.ms_devices:
            return

        ms_states = []
        if setting == 1: ms_states = [False, False, False]
        elif setting == 2: ms_states = [True, False, False]
        elif setting == 4: ms_states = [False, True, False]
        elif setting == 8: ms_states = [True, True, False]
        elif setting == 16: ms_states = [True, True, True]
        # For DRV8825:
        elif setting == 32 and len(self.ms_devices) == 3: ms_states = [True, True, True] # MS0, MS1, MS2 for DRV8825
        else:
            self.get_logger().warn(f"Unsupported microstep setting: {setting}. Defaulting to 1 (full step).")
            ms_states = [False, False, False]
            setting = 1

        for i, state in enumerate(ms_states):
            if i < len(self.ms_devices):
                if state:
                    self.ms_devices[i].on()
                else:
                    self.ms_devices[i].off()
        self.current_microstep = setting
        self.effective_steps_per_revolution = self.steps_per_revolution_fullstep * self.current_microstep

    def enable_motor(self):
        if self.enable_device:
            self.enable_device.off() # Active low
            self.get_logger().debug("A4988: Motor enabled.")

    def disable_motor(self):
        if self.enable_device:
            self.enable_device.on() # Active low
            self.get_logger().debug("A4988: Motor disabled.")

    def _calculate_delay_for_speed(self, speed_factor: float) -> float:
        """
        Calculates the required delay between steps to achieve a target speed.
        Speed factor is -1.0 to 1.0.
        """
        if speed_factor == 0.0:
            return 0.0 # Stop
        
        # Max steps per second at max RPM
        max_steps_per_sec = (self.motor_max_rpm / 60.0) * self.effective_steps_per_revolution
        
        # Target steps per second
        target_steps_per_sec = abs(speed_factor) * max_steps_per_sec
        
        if target_steps_per_sec == 0:
            return 0.0
            
        # Delay = 1 / steps_per_sec
        calculated_delay = 1.0 / target_steps_per_sec
        
        # Ensure minimum delay is respected
        return max(calculated_delay, self.step_delay_s)


    def _motor_control_thread(self):
        """Thread for executing steps based on target_steps or current_speed_factor."""
        self.get_logger().info("A4988: Motor control thread started.")
        self.enable_motor()

        while not self.stop_event.is_set():
            delay = 0.0
            with self.control_lock:
                if self.target_steps != 0:
                    direction = 1 if self.target_steps > 0 else -1
                    self.dir_device.value = (direction > 0) # True for forward (CW)
                    delay = self._calculate_delay_for_speed(1.0) # Move at max speed
                    
                    if delay > 0:
                        self.step_device.on()
                        time.sleep(delay / 2.0)
                        self.step_device.off()
                        time.sleep(delay / 2.0)
                        self.target_steps -= direction
                    
                elif self.current_speed_factor != 0.0:
                    self.dir_device.value = (self.current_speed_factor > 0)
                    delay = self._calculate_delay_for_speed(self.current_speed_factor)
                    
                    if delay > 0:
                        self.step_device.on()
                        time.sleep(delay / 2.0)
                        self.step_device.off()
                        time.sleep(delay / 2.0)
                else:
                    self._stop_motor() # All pins off
                    self.disable_motor() # Disable driver if not moving
                    # Use a small sleep to avoid busy-waiting when idle
                    time.sleep(0.01) # Small sleep when idle
                    
            if delay == 0 and (self.target_steps == 0 and self.current_speed_factor == 0.0):
                # Ensure a small sleep if motor is completely idle to prevent 100% CPU usage
                time.sleep(0.01)

        self.get_logger().info("A4988: Motor control thread stopped.")
        self.disable_motor() # Ensure motor disabled on thread exit


    def cmd_steps_callback(self, msg: Int32):
        """Moves the motor by a specified number of steps."""
        with self.control_lock:
            self.target_steps += msg.data
            self.current_speed_factor = 0.0 # Stop continuous motion
        self.get_logger().info(f'A4988: Received steps command: {msg.data}. Target steps: {self.target_steps}')
        if not self.motor_thread or not self.motor_thread.is_alive():
            self.motor_thread = threading.Thread(target=self._motor_control_thread)
            self.motor_thread.start()

    def cmd_speed_callback(self, msg: Float32):
        """Sets the motor speed for continuous rotation."""
        speed = max(-1.0, min(1.0, msg.data)) # Clamp speed
        with self.control_lock:
            self.current_speed_factor = speed
            self.target_steps = 0 # Stop step motion
        self.get_logger().info(f'A4988: Received speed command: {self.current_speed_factor}')
        if not self.motor_thread or not self.motor_thread.is_alive():
            self.motor_thread = threading.Thread(target=self._motor_control_thread)
            self.motor_thread.start()

    def destroy_node(self):
        self.stop_event.set()
        if self.motor_thread and self.motor_thread.is_alive():
            self.motor_thread.join(timeout=1.0) # Wait for thread to finish
        self.disable_motor() # Ensure motor disabled
        if self.step_device: self.step_device.close()
        if self.dir_device: self.dir_device.close()
        if self.enable_device: self.enable_device.close()
        for device in self.ms_devices: device.close()
        self.get_logger().info('A4988: GPIOs released.')
        super().destroy_node()

# Simple MockDigitalOutputDevice for when GPIO initialization fails in mock mode
class MockDigitalOutputDevice:
    def __init__(self, pin, initial_value=False):
        self.pin = pin
        self.value = initial_value
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
    node = A4988DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
