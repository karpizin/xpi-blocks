#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String, UInt8, Int32
from pyubx2 import UBXReader
import serial
import threading

class GpsRtkNode(Node):
    """
    ROS2 Driver for RTK-capable GPS modules (e.g. u-blox ZED-F9P).
    Handles high-precision positioning and RTCM correction injection.
    """
    def __init__(self):
        super().__init__('gps_rtk_node')

        # 1. Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('frame_id', 'gps_link')
        
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value

        # 2. Publishers
        self.fix_pub = self.create_publisher(NavSatFix, '~/fix', 10)
        self.status_pub = self.create_publisher(Int32, '~/rtk_status', 10) # 0=No, 1=3D, 2=Float, 3=Fixed

        # 3. Subscribers (For RTCM corrections)
        self.create_subscription(String, '/rtk/corrections', self.correction_callback, 10)

        # 4. Serial Init
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.ubr = UBXReader(self.ser)
            self.get_logger().info(f"Connected to RTK GPS on {self.port} at {self.baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # 5. Reader Thread
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def correction_callback(self, msg):
        """Injects RTCM corrections from topic into the hardware serial port."""
        if hasattr(self, 'ser') and self.ser.is_open:
            # Assuming payload is hex or raw bytes. For simplicity, we expect raw bytes string.
            try:
                data = bytes.fromhex(msg.data)
                self.ser.write(data)
            except:
                # If not hex, try raw
                self.ser.write(msg.data.encode('utf-8'))

    def read_loop(self):
        """Continuously reads UBX/NMEA messages from the serial port."""
        while rclpy.ok():
            try:
                (raw_data, parsed_data) = self.ubr.read()
                if parsed_data:
                    # Look for UBX-NAV-PVT (Position, Velocity, Time)
                    if parsed_data.identity == "NAV-PVT":
                        self.publish_fix(parsed_data)
            except Exception as e:
                self.get_logger().warn(f"Read error: {e}")

    def publish_fix(self, data):
        """Converts UBX NAV-PVT data to ROS2 NavSatFix."""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # UBX data is in 1e-7 degrees
        msg.latitude = data.lat * 1e-7
        msg.longitude = data.lon * 1e-7
        msg.altitude = float(data.hMSL * 1e-3) # Height above Mean Sea Level in meters

        # RTK Status Mapping
        # carrSoln: 0 = no carrier phase, 1 = float, 2 = fixed
        # fixType: 3 = 3D fix
        rtk_flag = data.carrSoln
        
        if rtk_flag == 2: # FIXED
            msg.status.status = NavSatStatus.STATUS_GBAS_FIX
            status_val = 3
        elif rtk_flag == 1: # FLOAT
            msg.status.status = NavSatStatus.STATUS_FIX
            status_val = 2
        else: # Standard 3D or No Fix
            msg.status.status = NavSatStatus.STATUS_FIX if data.fixType >= 3 else NavSatStatus.STATUS_NO_FIX
            status_val = 1 if data.fixType >= 3 else 0

        self.fix_pub.publish(msg)
        self.status_pub.publish(Int32(data=status_val))

def main(args=None):
    rclpy.init(args=args)
    node = GpsRtkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()