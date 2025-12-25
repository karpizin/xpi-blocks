#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from xpi_commons.i2c_helper import get_smbus
import time

class PCA9685MotorNode(Node):
    """
    ROS2 Node for controlling DC motors via PCA9685 I2C PWM Driver.
    Commonly used in Waveshare Motor Driver HAT and similar.
    Each motor uses 3 channels on PCA9685: 1 for PWM, 2 for Direction (IN1/IN2).
    """
    # PCA9685 Registers
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self):
        super().__init__('pca9685_motor_node')

        # 1. Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('pwm_frequency', 1000.0)
        
        # Motor A mapping (Channels on PCA9685)
        self.declare_parameter('motor_a_pwm', 0)
        self.declare_parameter('motor_a_in1', 1)
        self.declare_parameter('motor_a_in2', 2)
        
        # Motor B mapping
        self.declare_parameter('motor_b_pwm', 5)
        self.declare_parameter('motor_b_in1', 3)
        self.declare_parameter('motor_b_in2', 4)

        self.declare_parameter('mock_hardware', False)

        # 2. Config
        self.bus_id = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.freq = self.get_parameter('pwm_frequency').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        # 3. Init I2C and PCA9685
        self.bus = get_smbus(self.bus_id, mock=self.mock_mode)
        self._init_pca9685()

        # 4. Subscribers
        self.create_subscription(Float32, '~/motor_a/cmd_speed', self.motor_a_callback, 10)
        self.create_subscription(Float32, '~/motor_b/cmd_speed', self.motor_b_callback, 10)

        self.get_logger().info(f'PCA9685 Motor Driver initialized at 0x{self.address:02X}')

    def _init_pca9685(self):
        if self.mock_mode: return
        try:
            self.bus.write_byte_data(self.address, self.MODE1, 0x00)
            prescale = int(25000000.0 / (4096.0 * self.freq) - 1 + 0.5)
            old_mode = self.bus.read_byte_data(self.address, self.MODE1)
            self.bus.write_byte_data(self.address, self.MODE1, (old_mode & 0x7F) | 0x10)
            self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
            self.bus.write_byte_data(self.address, self.MODE1, old_mode)
            time.sleep(0.005)
            self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0xA0)
        except Exception as e:
            self.get_logger().error(f'Failed to init PCA9685: {e}')

    def _set_pwm(self, channel, on, off):
        if self.mock_mode: return
        reg_base = self.LED0_ON_L + (4 * channel)
        self.bus.write_byte_data(self.address, reg_base, on & 0xFF)
        self.bus.write_byte_data(self.address, reg_base + 1, on >> 8)
        self.bus.write_byte_data(self.address, reg_base + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, reg_base + 3, off >> 8)

    def _control_motor(self, pwm_ch, in1_ch, in2_ch, speed):
        # Clamp speed -1.0 to 1.0
        speed = max(-1.0, min(1.0, speed))
        
        duty = int(abs(speed) * 4095)
        
        if speed > 0: # Forward
            self._set_pwm(in1_ch, 4096, 0)
            self._set_pwm(in2_ch, 0, 4096)
        elif speed < 0: # Reverse
            self._set_pwm(in1_ch, 0, 4096)
            self._set_pwm(in2_ch, 4096, 0)
        else: # Stop
            self._set_pwm(in1_ch, 0, 4096)
            self._set_pwm(in2_ch, 0, 4096)
            duty = 0
            
        self._set_pwm(pwm_ch, 0, duty)

    def motor_a_callback(self, msg):
        self._control_motor(
            self.get_parameter('motor_a_pwm').value,
            self.get_parameter('motor_a_in1').value,
            self.get_parameter('motor_a_in2').value,
            msg.data
        )

    def motor_b_callback(self, msg):
        self._control_motor(
            self.get_parameter('motor_b_pwm').value,
            self.get_parameter('motor_b_in1').value,
            self.get_parameter('motor_b_in2').value,
            msg.data
        )

    def destroy_node(self):
        # Stop all motors
        for ch in range(16):
            self._set_pwm(ch, 0, 4096)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PCA9685MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
