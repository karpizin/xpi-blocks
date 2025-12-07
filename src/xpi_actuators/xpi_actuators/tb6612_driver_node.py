import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import PWMOutputDevice, DigitalOutputDevice, Device
from gpiozero.pins.mock import MockFactory
import os
import math

class TB6612DriverNode(Node):
    """
    ROS2 Node for controlling two DC motors via the TB6612FNG motor driver.
    Each motor receives a Float32 message for speed (-1.0 to 1.0).
    """

    def __init__(self):
        super().__init__('tb6612_driver_node')

        # 1. Declare Parameters
        # Motor A
        self.declare_parameter('motor_a_pwm_pin', 12)  # GPIO BCM
        self.declare_parameter('motor_a_in1_pin', 5)   # GPIO BCM
        self.declare_parameter('motor_a_in2_pin', 6)   # GPIO BCM
        # Motor B
        self.declare_parameter('motor_b_pwm_pin', 13)  # GPIO BCM
        self.declare_parameter('motor_b_in1_pin', 26)  # GPIO BCM
        self.declare_parameter('motor_b_in2_pin', 19)  # GPIO BCM

        self.declare_parameter('pwm_frequency', 1000)  # Hz
        self.declare_parameter('mock_hardware', False)

        # 2. Read Parameters
        # Motor A
        ma_pwm_pin = self.get_parameter('motor_a_pwm_pin').value
        ma_in1_pin = self.get_parameter('motor_a_in1_pin').value
        ma_in2_pin = self.get_parameter('motor_a_in2_pin').value
        # Motor B
        mb_pwm_pin = self.get_parameter('motor_b_pwm_pin').value
        mb_in1_pin = self.get_parameter('motor_b_in1_pin').value
        mb_in2_pin = self.get_parameter('motor_b_in2_pin').value

        pwm_freq = self.get_parameter('pwm_frequency').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        # Initialize mock factory if needed
        if self.mock_mode or os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock':
            self.get_logger().warn('TB6612: Running in MOCK mode. No real GPIO will be used.')
            Device.pin_factory = MockFactory()

        # 3. Hardware Init
        try:
            self.motor_a_pwm = PWMOutputDevice(ma_pwm_pin, frequency=pwm_freq, initial_value=0)
            self.motor_a_in1 = DigitalOutputDevice(ma_in1_pin, initial_value=False)
            self.motor_a_in2 = DigitalOutputDevice(ma_in2_pin, initial_value=False)
            self.get_logger().info(f'TB6612: Motor A initialized (PWM:{ma_pwm_pin}, IN1:{ma_in1_pin}, IN2:{ma_in2_pin})')

            self.motor_b_pwm = PWMOutputDevice(mb_pwm_pin, frequency=pwm_freq, initial_value=0)
            self.motor_b_in1 = DigitalOutputDevice(mb_in1_pin, initial_value=False)
            self.motor_b_in2 = DigitalOutputDevice(mb_in2_pin, initial_value=False)
            self.get_logger().info(f'TB6612: Motor B initialized (PWM:{mb_pwm_pin}, IN1:{mb_in1_pin}, IN2:{mb_in2_pin})')
        except Exception as e:
            self.get_logger().error(f'TB6612: Failed to initialize GPIO for motors: {e}')
            self.mock_mode = True # Fallback to mock if real GPIO fails
            # Create dummy devices for mock mode if not already done
            self.motor_a_pwm = MockOutputDevice()
            self.motor_a_in1 = MockOutputDevice()
            self.motor_a_in2 = MockOutputDevice()
            self.motor_b_pwm = MockOutputDevice()
            self.motor_b_in1 = MockOutputDevice()
            self.motor_b_in2 = MockOutputDevice()
            
        # 4. Subscribers
        self.sub_motor_a = self.create_subscription(
            Float32,
            '~/motor_a/cmd_vel',
            self.motor_a_callback,
            10
        )
        self.sub_motor_b = self.create_subscription(
            Float32,
            '~/motor_b/cmd_vel',
            self.motor_b_callback,
            10
        )
        self.get_logger().info('TB6612: Subscribing to motor commands.')

    def _set_motor_speed(self, pwm_device, in1_device, in2_device, speed: float):
        """
        Sets the speed and direction of a single motor.
        speed: -1.0 (full reverse) to 1.0 (full forward), 0.0 (stop)
        """
        speed = max(-1.0, min(1.0, speed)) # Clamp speed to -1.0 to 1.0

        if self.mock_mode:
            self.get_logger().debug(f'TB6612 Mock: Setting motor speed to {speed}')
            pwm_device.value = abs(speed) # MockOutputDevice will just log
            in1_device.value = (speed > 0)
            in2_device.value = (speed < 0)
            return

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

    def motor_a_callback(self, msg: Float32):
        self._set_motor_speed(self.motor_a_pwm, self.motor_a_in1, self.motor_a_in2, msg.data)

    def motor_b_callback(self, msg: Float32):
        self._set_motor_speed(self.motor_b_pwm, self.motor_b_in1, self.motor_b_in2, msg.data)

    def destroy_node(self):
        # Stop motors on shutdown
        self._set_motor_speed(self.motor_a_pwm, self.motor_a_in1, self.motor_a_in2, 0)
        self._set_motor_speed(self.motor_b_pwm, self.motor_b_in1, self.motor_b_in2, 0)
        
        # Close GPIO devices
        self.motor_a_pwm.close()
        self.motor_a_in1.close()
        self.motor_a_in2.close()
        self.motor_b_pwm.close()
        self.motor_b_in1.close()
        self.motor_b_in2.close()
        
        self.get_logger().info('TB6612: Motors stopped and GPIO closed.')
        super().destroy_node()

# Simple MockOutputDevice for when GPIO initialization fails in mock mode
class MockOutputDevice:
    def __init__(self, pin=None, initial_value=False, frequency=None):
        self.value = initial_value
        self.pin = pin
        self.frequency = frequency
        # logging.getLogger(__name__).debug(f'MockOutputDevice {pin} initialized with value {initial_value}')
    def on(self):
        self.value = True
        # logging.getLogger(__name__).debug(f'MockOutputDevice {self.pin} ON')
    def off(self):
        self.value = False
        # logging.getLogger(__name__).debug(f'MockOutputDevice {self.pin} OFF')
    def close(self):
        # logging.getLogger(__name__).debug(f'MockOutputDevice {self.pin} CLOSED')
        pass

def main(args=None):
    rclpy.init(args=args)
    node = TB6612DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
