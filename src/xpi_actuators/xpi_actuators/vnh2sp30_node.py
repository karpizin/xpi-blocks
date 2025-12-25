#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import PWMOutputDevice, DigitalOutputDevice, Device
from gpiozero.pins.mock import MockFactory
import os
import time

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
        self.declare_parameter('motor_a_en_pin', None) # Optional Enable/DIAG pin
        
        # Motor B
        self.declare_parameter('motor_b_pwm_pin', 13)
        self.declare_parameter('motor_b_in1_pin', 26)
        self.declare_parameter('motor_b_in2_pin', 19)
        self.declare_parameter('motor_b_en_pin', None)

        self.declare_parameter('pwm_frequency', 20000) # VNH2SP30 supports up to 20kHz (ultrasonic)
        self.declare_parameter('mock_hardware', False)

        # 2. Config
        self.mock_mode = self.get_parameter('mock_hardware').value
        if self.mock_mode or os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock':
            self.get_logger().warn('VNH2SP30: Running in MOCK mode.')
            Device.pin_factory = MockFactory()

        # 3. Hardware Init
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

            self.get_logger().info('VNH2SP30 High-Power Driver Initialized.')
        except Exception as e:
            self.get_logger().error(f'Failed to init VNH2SP30 GPIO: {e}')
            return

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
        speed = max(-1.0, min(1.0, speed))
        
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
        self._set_motor(self.mot_a_pwm, self.mot_a_in1, self.mot_a_in2, msg.data)

    def motor_b_callback(self, msg):
        self._set_motor(self.mot_b_pwm, self.mot_b_in1, self.mot_b_in2, msg.data)

    def destroy_node(self):
        self._set_motor(self.mot_a_pwm, self.mot_a_in1, self.mot_a_in2, 0)
        self._set_motor(self.mot_b_pwm, self.mot_b_in1, self.mot_b_in2, 0)
        if self.mot_a_en: self.mot_a_en.off()
        if self.mot_b_en: self.mot_b_en.off()
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
