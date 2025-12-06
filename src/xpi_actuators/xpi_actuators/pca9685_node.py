import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from xpi_commons.i2c_helper import get_smbus
import math
import time

class PCA9685Node(Node):
    """
    Driver for PCA9685 16-Channel PWM Driver.
    Supports Servo mode (50Hz) and Generic PWM mode.
    """
    # Registers
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self):
        super().__init__('pca9685_node')

        # 1. Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('frequency', 50.0)  # 50Hz for Servos
        self.declare_parameter('mock_hardware', False)
        
        # 2. Config
        self.bus_id = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.freq = self.get_parameter('frequency').value
        mock_mode = self.get_parameter('mock_hardware').value

        # 3. Init I2C
        self.bus = get_smbus(self.bus_id, mock=mock_mode)
        self.init_pca9685()

        # 4. Subscriber
        # Expects array of values. Index 0 -> Channel 0.
        # Values: 0.0 to 1.0 (PWM duty cycle)
        self.sub = self.create_subscription(
            Float32MultiArray,
            '~/cmd',
            self.cmd_callback,
            10
        )
        self.get_logger().info(f'PCA9685 initialized at 0x{self.address:02X} on bus {self.bus_id} ({self.freq}Hz)')

    def init_pca9685(self):
        """Reset and set frequency."""
        try:
            # 1. Reset (Sleep mode)
            self.bus.write_byte_data(self.address, self.MODE1, 0x00)
            
            # 2. Set Frequency
            # Formula: prescale = round(osc_clock / (4096 * update_rate)) - 1
            # osc_clock = 25MHz
            prescale_val = int(25000000.0 / (4096.0 * self.freq) - 1 + 0.5)
            
            # Go to sleep to set prescale
            old_mode = self.bus.read_byte_data(self.address, self.MODE1)
            new_mode = (old_mode & 0x7F) | 0x10  # Sleep bit
            self.bus.write_byte_data(self.address, self.MODE1, new_mode)
            self.bus.write_byte_data(self.address, self.PRESCALE, prescale_val)
            self.bus.write_byte_data(self.address, self.MODE1, old_mode)
            
            time.sleep(0.005)
            # Enable auto-increment
            self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0xA0) 
        except Exception as e:
            self.get_logger().error(f'Failed to configure PCA9685: {e}')

    def set_pwm(self, channel, on, off):
        """Write to LEDn registers."""
        # 4 registers per channel: ON_L, ON_H, OFF_L, OFF_H
        reg_base = self.LED0_ON_L + (4 * channel)
        try:
            self.bus.write_byte_data(self.address, reg_base, on & 0xFF)
            self.bus.write_byte_data(self.address, reg_base + 1, on >> 8)
            self.bus.write_byte_data(self.address, reg_base + 2, off & 0xFF)
            self.bus.write_byte_data(self.address, reg_base + 3, off >> 8)
        except Exception as e:
            self.get_logger().error(f'I2C Error ch{channel}: {e}')

    def cmd_callback(self, msg):
        """
        Expects normalized input [0.0 ... 1.0] for each channel.
        For servos: 0.0 -> 0% duty (Off), 0.1 -> ~2ms? No.
        Let's stick to pure PWM duty cycle (0.0-1.0) for this base driver.
        Servo math (angle -> duty) should be done by the sender or a helper.
        """
        for i, val in enumerate(msg.data):
            if i > 15: break
            
            # Clamp 0.0 - 1.0
            val = max(0.0, min(1.0, val))
            
            if val == 0.0:
                self.set_pwm(i, 0, 4096) # Full OFF
            elif val == 1.0:
                self.set_pwm(i, 4096, 0) # Full ON
            else:
                off_tick = int(val * 4095)
                self.set_pwm(i, 0, off_tick)

def main(args=None):
    rclpy.init(args=args)
    node = PCA9685Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Optional: Turn off all outputs on exit?
        # node.set_all_pwm(0, 0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
