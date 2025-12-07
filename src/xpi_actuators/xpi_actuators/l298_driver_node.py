import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from gpiozero import DigitalOutputDevice, PWMOutputDevice, Device
from gpiozero.pins.mock import MockFactory
import os
import time
import threading

class L298DriverNode(Node):
    """
    ROS2 Node for controlling a bipolar stepper motor or two DC motors via L298/L293 driver.
    Supports step control for stepper and speed/direction for DC motors.
    """

    def __init__(self):
        super().__init__('l298_driver_node')

        # 1. Declare Parameters
        self.declare_parameter('driver_mode', 'stepper') # 'stepper' or 'dc_dual'
        
        # Stepper mode pins (A4988 uses 2, L298 uses 4)
        self.declare_parameter('stepper_in1_pin', 17) # GPIO BCM
        self.declare_parameter('stepper_in2_pin', 27) # GPIO BCM
        self.declare_parameter('stepper_in3_pin', 22) # GPIO BCM
        self.declare_parameter('stepper_in4_pin', 23) # GPIO BCM
        self.declare_parameter('stepper_enable_pin', None) # Optional, for L298EN (active high)
        self.declare_parameter('stepper_steps_per_revolution', 200) # For typical 1.8 degree stepper
        self.declare_parameter('stepper_step_delay_s', 0.005) # Minimum delay in seconds between steps
        self.declare_parameter('stepper_motor_max_rpm', 60) # For continuous speed calculation

        # DC Dual Motor mode pins
        self.declare_parameter('motor_a_in1_pin', 17)
        self.declare_parameter('motor_a_in2_pin', 27)
        self.declare_parameter('motor_a_pwm_pin', None) # Optional, if using L298EN for speed
        
        self.declare_parameter('motor_b_in1_pin', 22)
        self.declare_parameter('motor_b_in2_pin', 23)
        self.declare_parameter('motor_b_pwm_pin', None) # Optional
        self.declare_parameter('dc_pwm_frequency', 1000) # Hz for DC motor speed

        self.declare_parameter('mock_hardware', False)

        # 2. Read Parameters
        self.driver_mode = self.get_parameter('driver_mode').value
        self.mock_mode = self.get_parameter('mock_hardware').value
        
        # Initialize mock factory if needed
        if self.mock_mode or os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock':
            self.get_logger().warn('L298: Running in MOCK mode. No real GPIO will be used.')
            Device.pin_factory = MockFactory()

        self.pins = [] # Store all GPIO devices
        self.control_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.motor_thread = None

        if self.driver_mode == 'stepper':
            self._init_stepper_mode()
        elif self.driver_mode == 'dc_dual':
            self._init_dc_dual_mode()
        else:
            self.get_logger().error(f"L298: Invalid driver_mode: {self.driver_mode}. Must be 'stepper' or 'dc_dual'.")
            rclpy.shutdown()
            return
        
        self.get_logger().info(f'L298: Subscribing to motor commands in {self.driver_mode} mode.')

    def _init_stepper_mode(self):
        self.stepper_in_pins = [
            self.get_parameter('stepper_in1_pin').value,
            self.get_parameter('stepper_in2_pin').value,
            self.get_parameter('stepper_in3_pin').value,
            self.get_parameter('stepper_in4_pin').value
        ]
        self.stepper_enable_pin = self.get_parameter('stepper_enable_pin').value
        self.stepper_steps_per_revolution = self.get_parameter('stepper_steps_per_revolution').value
        self.stepper_step_delay_s = self.get_parameter('stepper_step_delay_s').value
        self.stepper_motor_max_rpm = self.get_parameter('stepper_motor_max_rpm').value

        # Full step sequence for 4-phase bipolar/unipolar motor with L298
        self.step_sequence = [
            [1, 0, 1, 0], # A=on, B=on (simplified for 2 coils)
            [0, 1, 1, 0],
            [0, 1, 0, 1],
            [1, 0, 0, 1]
        ]
        self.step_index = 0

        try:
            for pin in self.stepper_in_pins:
                self.pins.append(DigitalOutputDevice(pin, initial_value=False))
            self.stepper_enable_device = None
            if self.stepper_enable_pin is not None:
                self.stepper_enable_device = DigitalOutputDevice(self.stepper_enable_pin, initial_value=False) # Active high
                self.pins.append(self.stepper_enable_device)
                self.stepper_enable_device.on() # Enable the driver
            self.get_logger().info(f'L298 Stepper: Initialized on GPIOs {self.stepper_in_pins}.')
        except Exception as e:
            self.get_logger().error(f'L298 Stepper: Failed to initialize GPIO: {e}. Falling back to mock.')
            self.mock_mode = True
            for pin in self.stepper_in_pins:
                self.pins.append(MockDigitalOutputDevice(pin))
            if self.stepper_enable_pin is not None:
                self.stepper_enable_device = MockDigitalOutputDevice(self.stepper_enable_pin)
                self.pins.append(self.stepper_enable_device)

        self.target_steps = 0
        self.current_speed_factor = 0.0 # -1.0 to 1.0 (relative to max RPM)

        self.steps_sub = self.create_subscription(Int32, '~/cmd_steps', self.cmd_stepper_steps_callback, 10)
        self.speed_sub = self.create_subscription(Float32, '~/cmd_speed', self.cmd_stepper_speed_callback, 10)

    def _init_dc_dual_mode(self):
        self.motor_a_in1_pin = self.get_parameter('motor_a_in1_pin').value
        self.motor_a_in2_pin = self.get_parameter('motor_a_in2_pin').value
        self.motor_a_pwm_pin = self.get_parameter('motor_a_pwm_pin').value
        
        self.motor_b_in1_pin = self.get_parameter('motor_b_in1_pin').value
        self.motor_b_in2_pin = self.get_parameter('motor_b_in2_pin').value
        self.motor_b_pwm_pin = self.get_parameter('motor_b_pwm_pin').value

        self.dc_pwm_frequency = self.get_parameter('dc_pwm_frequency').value

        try:
            self.motor_a_in1 = DigitalOutputDevice(self.motor_a_in1_pin, initial_value=False)
            self.motor_a_in2 = DigitalOutputDevice(self.motor_a_in2_pin, initial_value=False)
            self.pins.extend([self.motor_a_in1, self.motor_a_in2])
            if self.motor_a_pwm_pin is not None:
                self.motor_a_pwm = PWMOutputDevice(self.motor_a_pwm_pin, frequency=self.dc_pwm_frequency, initial_value=0)
                self.pins.append(self.motor_a_pwm)
            else:
                self.motor_a_pwm = None # Control speed by directly setting In1/In2
            
            self.motor_b_in1 = DigitalOutputDevice(self.motor_b_in1_pin, initial_value=False)
            self.motor_b_in2 = DigitalOutputDevice(self.motor_b_in2_pin, initial_value=False)
            self.pins.extend([self.motor_b_in1, self.motor_b_in2])
            if self.motor_b_pwm_pin is not None:
                self.motor_b_pwm = PWMOutputDevice(self.motor_b_pwm_pin, frequency=self.dc_pwm_frequency, initial_value=0)
                self.pins.append(self.motor_b_pwm)
            else:
                self.motor_b_pwm = None

            self.get_logger().info('L298 DC Dual: Initialized.')
        except Exception as e:
            self.get_logger().error(f'L298 DC Dual: Failed to initialize GPIO: {e}. Falling back to mock.')
            self.mock_mode = True
            self.motor_a_in1 = MockDigitalOutputDevice(self.motor_a_in1_pin)
            self.motor_a_in2 = MockDigitalOutputDevice(self.motor_a_in2_pin)
            self.motor_a_pwm = MockPWMOutputDevice(self.motor_a_pwm_pin) if self.motor_a_pwm_pin else None
            self.motor_b_in1 = MockDigitalOutputDevice(self.motor_b_in1_pin)
            self.motor_b_in2 = MockDigitalOutputDevice(self.motor_b_in2_pin)
            self.motor_b_pwm = MockPWMOutputDevice(self.motor_b_pwm_pin) if self.motor_b_pwm_pin else None
            self.pins.extend([self.motor_a_in1, self.motor_a_in2, self.motor_b_in1, self.motor_b_in2])
            if self.motor_a_pwm: self.pins.append(self.motor_a_pwm)
            if self.motor_b_pwm: self.pins.append(self.motor_b_pwm)

        self.motor_a_sub = self.create_subscription(Float32, '~/motor_a/cmd_speed', self.cmd_motor_a_speed_callback, 10)
        self.motor_b_sub = self.create_subscription(Float32, '~/motor_b/cmd_speed', self.cmd_motor_b_speed_callback, 10)


    def _set_stepper_pins(self, step):
        """Sets the state of the motor driver pins according to the step sequence."""
        if self.mock_mode:
            # self.get_logger().debug(f'Mock Stepper: Step {step}')
            return
        for i in range(4):
            if self.step_sequence[step][i] == 1:
                self.pins[i].on()
            else:
                self.pins[i].off()

    def _stop_stepper(self):
        """Turns off all motor driver pins."""
        if self.pins:
            for i in range(4): # First four pins are for IN1-IN4
                self.pins[i].off()
            self.get_logger().debug("L298 Stepper: Motor stopped, all pins off.")

    def _step_stepper(self, direction):
        """Performs a single step in the given direction."""
        if direction > 0: # Clockwise
            self.step_index = (self.step_index + 1) % len(self.step_sequence)
        else: # Counter-clockwise
            self.step_index = (self.step_index - 1 + len(self.step_sequence)) % len(self.step_sequence)
        self._set_stepper_pins(self.step_index)
        time.sleep(self.stepper_step_delay_s)

    def _calculate_stepper_delay(self, speed_factor: float) -> float:
        """
        Calculates the required delay between steps to achieve a target speed.
        Speed factor is -1.0 to 1.0.
        """
        if speed_factor == 0.0:
            return 0.0
        
        # Max steps per second at max RPM
        max_steps_per_sec = (self.stepper_motor_max_rpm / 60.0) * self.stepper_steps_per_revolution
        
        # Target steps per second
        target_steps_per_sec = abs(speed_factor) * max_steps_per_sec
        
        if target_steps_per_sec == 0:
            return 0.0
            
        calculated_delay = 1.0 / target_steps_per_sec
        return max(calculated_delay, self.stepper_step_delay_s)


    def _stepper_control_thread(self):
        """Thread for executing steps based on target_steps or current_speed_factor."""
        self.get_logger().info("L298 Stepper: Control thread started.")
        if self.stepper_enable_device:
            self.stepper_enable_device.on() # Enable driver if present

        while not self.stop_event.is_set():
            delay = 0.0
            with self.control_lock:
                if self.target_steps != 0:
                    direction = 1 if self.target_steps > 0 else -1
                    delay = self._calculate_stepper_delay(1.0) # Move at max speed
                    
                    if delay > 0:
                        self._step_stepper(direction)
                        self.target_steps -= direction
                    
                elif self.current_speed_factor != 0.0:
                    direction = 1 if self.current_speed_factor > 0 else -1
                    delay = self._calculate_stepper_delay(self.current_speed_factor)
                    
                    if delay > 0:
                        self._step_stepper(direction)
                else:
                    self._stop_stepper()
                    if self.stepper_enable_device:
                        self.stepper_enable_device.off() # Disable driver if not moving
                    time.sleep(0.01) # Small sleep when idle
                    
            if delay == 0 and (self.target_steps == 0 and self.current_speed_factor == 0.0):
                time.sleep(0.01)

        self.get_logger().info("L298 Stepper: Control thread stopped.")
        self._stop_stepper()
        if self.stepper_enable_device:
            self.stepper_enable_device.off()


    def cmd_stepper_steps_callback(self, msg: Int32):
        with self.control_lock:
            self.target_steps += msg.data
            self.current_speed_factor = 0.0
        self.get_logger().info(f'L298 Stepper: Received steps command: {msg.data}. Target steps: {self.target_steps}')
        if not self.motor_thread or not self.motor_thread.is_alive():
            self.motor_thread = threading.Thread(target=self._stepper_control_thread)
            self.motor_thread.start()

    def cmd_stepper_speed_callback(self, msg: Float32):
        speed = max(-1.0, min(1.0, msg.data))
        with self.control_lock:
            self.current_speed_factor = speed
            self.target_steps = 0
        self.get_logger().info(f'L298 Stepper: Received speed command: {self.current_speed_factor}')
        if not self.motor_thread or not self.motor_thread.is_alive():
            self.motor_thread = threading.Thread(target=self._stepper_control_thread)
            self.motor_thread.start()

    def _set_dc_motor_speed(self, in1_device, in2_device, pwm_device, speed: float):
        speed = max(-1.0, min(1.0, speed))

        if self.mock_mode:
            self.get_logger().debug(f'L298 Mock DC: Setting motor speed to {speed}')
            return

        if pwm_device: # If PWM pin is provided (L298EN for speed)
            if speed > 0: # Forward
                in1_device.on()
                in2_device.off()
                pwm_device.value = speed
            elif speed < 0: # Reverse
                in1_device.off()
                in2_device.on()
                pwm_device.value = abs(speed)
            else: # Stop
                in1_device.off()
                in2_device.off()
                pwm_device.value = 0
        else: # No PWM pin, control speed by direct on/off (less precise)
            if speed > 0: # Forward
                in1_device.on()
                in2_device.off()
            elif speed < 0: # Reverse
                in1_device.off()
                in2_device.on()
            else: # Stop
                in1_device.off()
                in2_device.off()

    def cmd_motor_a_speed_callback(self, msg: Float32):
        self._set_dc_motor_speed(self.motor_a_in1, self.motor_a_in2, self.motor_a_pwm, msg.data)

    def cmd_motor_b_speed_callback(self, msg: Float32):
        self._set_dc_motor_speed(self.motor_b_in1, self.motor_b_in2, self.motor_b_pwm, msg.data)


    def destroy_node(self):
        self.stop_event.set()
        if self.motor_thread and self.motor_thread.is_alive():
            self.motor_thread.join(timeout=1.0)
        
        if self.driver_mode == 'stepper':
            self._stop_stepper()
            if self.stepper_enable_device: self.stepper_enable_device.off()
        elif self.driver_mode == 'dc_dual':
            self._set_dc_motor_speed(self.motor_a_in1, self.motor_a_in2, self.motor_a_pwm, 0)
            self._set_dc_motor_speed(self.motor_b_in1, self.motor_b_in2, self.motor_b_pwm, 0)
        
        for pin_device in self.pins:
            pin_device.close()
        
        self.get_logger().info('L298: GPIOs released.')
        super().destroy_node()

# Simple MockDigitalOutputDevice and MockPWMOutputDevice for when GPIO initialization fails in mock mode
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

class MockPWMOutputDevice:
    def __init__(self, pin, frequency=1000, initial_value=0):
        self.pin = pin
        self.frequency = frequency
        self.value = initial_value
        # logging.getLogger(__name__).debug(f'MockPWMOutputDevice {pin} initialized')
    def close(self):
        pass
    @property
    def value(self):
        return self._value
    @value.setter
    def value(self, val):
        self._value = val

def main(args=None):
    rclpy.init(args=args)
    node = L298DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
