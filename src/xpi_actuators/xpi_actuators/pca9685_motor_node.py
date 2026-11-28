#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from xpi_commons.i2c_helper import get_smbus
import time

from xpi_actuators.motor_safety import DEFAULT_CMD_TIMEOUT_S, CommandWatchdog, sanitize_speed

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
        self.declare_parameter('cmd_timeout', DEFAULT_CMD_TIMEOUT_S)  # s; stop a motor if no command arrives in time (0 = off)
        self.declare_parameter('max_speed', 1.0)  # absolute speed limit applied to every command (0..1)

        # 2. Config
        self.bus_id = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.freq = self.get_parameter('pwm_frequency').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        # 3. Init I2C and PCA9685
        self.bus = get_smbus(self.bus_id, mock=self.mock_mode)
        self._init_pca9685()

        # Dead-man switch: publishers must stream commands; a silent channel is stopped.
        self.max_speed = sanitize_speed(self.get_parameter('max_speed').value)
        self.watchdog = CommandWatchdog(self.get_parameter('cmd_timeout').value)
        self._stop_channel = {
            'motor_a': lambda: self._drive('motor_a', 0.0),
            'motor_b': lambda: self._drive('motor_b', 0.0),
        }
        if self.watchdog.enabled:
            self.create_timer(self.watchdog.check_period(), self._watchdog_tick)
        else:
            self.get_logger().warn('PCA9685 Motor: cmd_timeout <= 0, command watchdog DISABLED.')

        # 4. Subscribers
        self.create_subscription(Float32, '~/motor_a/cmd_speed', self.motor_a_callback, 10)
        self.create_subscription(Float32, '~/motor_b/cmd_speed', self.motor_b_callback, 10)

        self.get_logger().info(f'PCA9685 Motor Driver initialized at 0x{self.address:02X}')

    def _init_pca9685(self):
        if self.mock_mode:
            return
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
        if self.mock_mode:
            return
        reg_base = self.LED0_ON_L + (4 * channel)
        # One transaction (MODE1 auto-increment is on): the output never sees a
        # half-updated ON/OFF pair, which made servos twitch.
        self.bus.write_i2c_block_data(
            self.address, reg_base,
            [on & 0xFF, (on >> 8) & 0x1F, off & 0xFF, (off >> 8) & 0x1F])

    def _control_motor(self, pwm_ch, in1_ch, in2_ch, speed):
        speed = sanitize_speed(speed, self.max_speed)  # clamp; NaN/inf -> stop
        
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

    def _drive(self, motor, speed):
        self._control_motor(
            self.get_parameter(f'{motor}_pwm').value,
            self.get_parameter(f'{motor}_in1').value,
            self.get_parameter(f'{motor}_in2').value,
            speed
        )

    def motor_a_callback(self, msg):
        speed = sanitize_speed(msg.data, self.max_speed)
        self.watchdog.feed('motor_a', speed)
        self._drive('motor_a', speed)

    def motor_b_callback(self, msg):
        speed = sanitize_speed(msg.data, self.max_speed)
        self.watchdog.feed('motor_b', speed)
        self._drive('motor_b', speed)

    def _watchdog_tick(self):
        for channel in self.watchdog.expired():
            self.get_logger().warn(
                f'PCA9685 Motor: no command on {channel} for {self.watchdog.timeout:.2f} s, stopping motor.')
            self._stop_channel[channel]()

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
