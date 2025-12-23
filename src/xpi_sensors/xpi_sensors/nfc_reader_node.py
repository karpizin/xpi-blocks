#!/usr/bin/env python3
import time
import board
import busio
from digitalio import DigitalInOut
from adafruit_pn532.i2c import PN532_I2C

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class NFCReaderNode(Node):
    """
    ROS2 Node for PN532 NFC/RFID Reader via I2C.
    Publishes UID of detected tags.
    """
    def __init__(self):
        super().__init__('nfc_reader_node')
        
        # Parameters
        self.declare_parameter('polling_rate', 2.0) # Hz
        self.declare_parameter('reset_pin', 4) # GPIO for Reset (Optional)
        self.declare_parameter('req_pin', 12)  # GPIO for Request (Optional)
        self.declare_parameter('frame_id', 'nfc_link')
        
        self.polling_rate = self.get_parameter('polling_rate').value
        # Note: PN532 I2C usually doesn't need explicit Reset/Req pins if wired simply, 
        # but the library supports them for stability.
        
        # Publishers
        self.uid_pub = self.create_publisher(String, '~/tag_uid', 10)
        self.detected_pub = self.create_publisher(Bool, '~/tag_detected', 10)

        self.pn532 = None
        self.last_uid = None

        try:
            i2c = board.I2C()
            # PN532 I2C Address is typically 0x24
            self.pn532 = PN532_I2C(i2c, debug=False)
            
            # Configure
            ic, ver, rev, support = self.pn532.firmware_version
            self.get_logger().info(f"Found PN532 (Firmware v{ver}.{rev})")
            
            self.pn532.SAM_configuration() # Configure standard reader mode
            
        except Exception as e:
            self.get_logger().error(f"Failed to init PN532: {e}")
            self.pn532 = None

        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)

    def timer_callback(self):
        if not self.pn532:
            return

        try:
            # Check for card (timeout=0.5s)
            uid = self.pn532.read_passive_target(timeout=0.1)
            
            if uid:
                # Convert bytearray to Hex String (e.g. "4A 3C 1D ...")
                uid_hex = " ".join([hex(i) for i in uid]).upper().replace("0X", "")
                
                # Only publish log if new tag
                if uid != self.last_uid:
                    self.get_logger().info(f"Tag Detected: {uid_hex}")
                    self.last_uid = uid
                
                self.uid_pub.publish(String(data=uid_hex))
                self.detected_pub.publish(Bool(data=True))
            else:
                self.last_uid = None
                self.detected_pub.publish(Bool(data=False))

        except RuntimeError as e:
            # "Response too short" etc. can happen if I2C bus is noisy
            self.get_logger().debug(f"NFC Read Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Critical NFC Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NFCReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
