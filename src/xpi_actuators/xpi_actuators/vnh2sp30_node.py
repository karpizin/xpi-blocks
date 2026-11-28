#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float32
from gpiozero import PWMOutputDevice, DigitalOutputDevice, Device
from gpiozero.pins.mock import MockFactory, MockPWMPin
import os
from xpi_commons.hardware import hardware_unavailable

from xpi_actuators.motor_safety import DEFAULT_CMD_TIMEOUT_S, CommandWatchdog, sanitize_speed

class VNH2SP30Node(Node):
    """
    ROS2 Node for the VNH2SP30 High-Power Motor Driver.
    Supports dual motor configuration (Monster Moto Shield style).
    Logic: 1 PWM pin for speed, 2 IN pins for direction, 1 EN pin for status/enable.
    """

    def __init__(self):
        super().__init__('vnh2sp30_node')

        # 1. Parameters
        # Motor A
        self.declare_parameter('motor_a_pwm_pin', 12)
        self.declare_parameter('motor_a_in1_pin', 5)
        self.declare_parameter('motor_a_in2_pin', 6)
        # Optional pins default to None; dynamic typing lets get_parameter() return None
        # (a statically typed None parameter raises ParameterUninitializedException).
        self.declare_parameter('motor_a_en_pin', None, ParameterDescriptor(dynamic_typing=True)) # Optional Enable/DIAG pin
        
        # Motor B
        self.declare_parameter('motor_b_pwm_pin', 13)
        self.declare_parameter('motor_b_in1_pin', 26)
        self.declare_parameter('motor_b_in2_pin', 19)
        self.declare_parameter('motor_b_en_pin', None, ParameterDescriptor(dynamic_typing=True))

        self.declare_parameter('pwm_frequency', 20000) # VNH2SP30 supports up to 20kHz (ultrasonic)
        self.declare_parameter('mock_hardware', False)
        self.declare_parameter('cmd_timeout', DEFAULT_CMD_TIMEOUT_S)  # s; stop a motor if no command arrives in time (0 = off)
        self.declare_parameter('max_speed', 1.0)  # absolute speed limit applied to every command (0..1)

        # 2. Config
        self.mock_mode = self.get_parameter('mock_hardware').value
        if self.mock_mode or os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock':
            self.get_logger().warn('VNH2SP30: Running in MOCK mode.')
            Device.pin_factory = MockFactory(pin_class=MockPWMPin)  # PWM-capable mock pins

        # 3. Hardware Init
        self.initialized = False
        self.mot_a_en = None
        self.mot_b_en = None
        try:
            # Motor A
            self.mot_a_pwm = PWMOutputDevice(self.get_parameter('motor_a_pwm_pin').value, frequency=self.get_parameter('pwm_frequency').value)
            self.mot_a_in1 = DigitalOutputDevice(self.get_parameter('motor_a_in1_pin').value)
            self.mot_a_in2 = DigitalOutputDevice(self.get_parameter('motor_a_in2_pin').value)
            
            # Motor B
            self.mot_b_pwm = PWMOutputDevice(self.get_parameter('motor_b_pwm_pin').value, frequency=self.get_parameter('pwm_frequency').value)
            self.mot_b_in1 = DigitalOutputDevice(self.get_parameter('motor_b_in1_pin').value)
            self.mot_b_in2 = DigitalOutputDevice(self.get_parameter('motor_b_in2_pin').value)

            # Optional Enable Pins
            self.mot_a_en = self._init_en_pin('motor_a_en_pin')
            self.mot_b_en = self._init_en_pin('motor_b_en_pin')

            self.initialized = True
            self.get_logger().info('VNH2SP30 High-Power Driver Initialized.')
        except Exception as e:
            if not self.mock_mode:
                hardware_unavailable(self, 'VNH2SP30 motor GPIO', e)
            self.get_logger().error(f'Failed to init VNH2SP30 GPIO: {e}')
            return

        # Dead-man switch: publishers must stream commands; a silent channel is stopped.
        self.max_speed = sanitize_speed(self.get_parameter('max_speed').value)
        self.watchdog = CommandWatchdog(self.get_parameter('cmd_timeout').value)
        self._stop_channel = {
            'motor_a': lambda: self._set_motor(self.mot_a_pwm, self.mot_a_in1, self.mot_a_in2, 0.0),
            'motor_b': lambda: self._set_motor(self.mot_b_pwm, self.mot_b_in1, self.mot_b_in2, 0.0),
        }
        if self.watchdog.enabled:
            self.create_timer(self.watchdog.check_period(), self._watchdog_tick)
        else:
            self.get_logger().warn('VNH2SP30: cmd_timeout <= 0, command watchdog DISABLED.')

        # 4. Subscribers
        self.create_subscription(Float32, '~/motor_a/cmd_speed', self.motor_a_callback, 10)
        self.create_subscription(Float32, '~/motor_b/cmd_speed', self.motor_b_callback, 10)

    def _init_en_pin(self, param_name):
        pin = self.get_parameter(param_name).value
        if pin is not None:
            dev = DigitalOutputDevice(pin)
            dev.on() # Usually active high to enable
            return dev
        return None

    def _set_motor(self, pwm_dev, in1_dev, in2_dev, speed):
        speed = sanitize_speed(speed, self.max_speed)  # clamp; NaN/inf -> stop

        if speed > 0: # Forward
            in1_dev.on()
            in2_dev.off()
            pwm_dev.value = speed
        elif speed < 0: # Reverse
            in1_dev.off()
            in2_dev.on()
            pwm_dev.value = abs(speed)
        else: # Stop
            in1_dev.off()
            in2_dev.off()
            pwm_dev.value = 0

    def motor_a_callback(self, msg):
        speed = sanitize_speed(msg.data, self.max_speed)
        self.watchdog.feed('motor_a', speed)
        self._set_motor(self.mot_a_pwm, self.mot_a_in1, self.mot_a_in2, speed)

    def motor_b_callback(self, msg):
        speed = sanitize_speed(msg.data, self.max_speed)
        self.watchdog.feed('motor_b', speed)
        self._set_motor(self.mot_b_pwm, self.mot_b_in1, self.mot_b_in2, speed)

    def _watchdog_tick(self):
        for channel in self.watchdog.expired():
            self.get_logger().warn(
                f'VNH2SP30: no command on {channel} for {self.watchdog.timeout:.2f} s, stopping motor.')
            self._stop_channel[channel]()

    def destroy_node(self):
        if self.initialized:
            self._set_motor(self.mot_a_pwm, self.mot_a_in1, self.mot_a_in2, 0)
            self._set_motor(self.mot_b_pwm, self.mot_b_in1, self.mot_b_in2, 0)
        if self.mot_a_en:
            self.mot_a_en.off()
        if self.mot_b_en:
            self.mot_b_en.off()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VNH2SP30Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
