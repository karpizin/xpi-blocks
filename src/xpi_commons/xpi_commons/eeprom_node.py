#!/usr/bin/env python3
import time
import board
import busio
import adafruit_24lc32 as adafruit_eeprom

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
from example_interfaces.srv import SetBool # We'll use a simple trigger for read all for now

class EEPROMNode(Node):
    """
    ROS2 Node for AT24Cxxx series I2C EEPROM.
    Provides raw access to non-volatile storage.
    """
    def __init__(self):
        super().__init__('eeprom_node')
        
        # Parameters
        self.declare_parameter('i2c_address', 0x50) # Standard for AT24C32/64/256
        self.declare_parameter('size', 32768) # Bits? No, library uses bytes. 32kbits = 4096 bytes.
        # AT24C32 = 4096 bytes, AT24C64 = 8192 bytes, AT24C256 = 32768 bytes.
        self.declare_parameter('eeprom_type', 'AT24C256') 
        
        self.address = self.get_parameter('i2c_address').value
        self.size = self.get_parameter('size').value
        
        # Publishers
        self.data_pub = self.create_publisher(UInt8MultiArray, '~/data_out', 10)
        
        # Subscribers
        self.create_subscription(UInt8MultiArray, '~/write_raw', self._write_callback, 10)
        
        self.eeprom = None
        try:
            i2c = board.I2C()
            # The library handles page writing and address sizing
            self.eeprom = adafruit_eeprom.EEPROM_I2C(i2c, address=self.address, size=self.size)
            self.get_logger().info(f"Initialized {self.get_parameter('eeprom_type').value} EEPROM ({self.size} bytes) at 0x{self.address:02X}")
        except Exception as e:
            self.get_logger().error(f"Failed to init EEPROM: {e}")

    def _write_callback(self, msg):
        """
        Expects: [AddrHigh, AddrLow, Data0, Data1, ...]
        """
        if not self.eeprom or len(msg.data) < 3:
            return
            
        try:
            addr = (msg.data[0] << 8) | msg.data[1]
            payload = bytes(msg.data[2:])
            
            self.get_logger().info(f"Writing {len(payload)} bytes to address 0x{addr:04X}")
            self.eeprom[addr:addr+len(payload)] = payload
            self.get_logger().info("Write successful.")
        except Exception as e:
            self.get_logger().error(f"EEPROM Write Error: {e}")

    def read_block(self, addr, length):
        if not self.eeprom: return None
        try:
            return self.eeprom[addr:addr+length]
        except Exception as e:
            self.get_logger().error(f"EEPROM Read Error: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = EEPROMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
