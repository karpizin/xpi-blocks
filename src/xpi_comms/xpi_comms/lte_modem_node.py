#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import NavSatFix
import serial
import time
import re

class LTEModemNode(Node):
    """
    ROS2 Node for SIM7600 series LTE Modems.
    Handles:
    - Signal Strength (RSSI) monitoring
    - SMS sending/receiving
    - GPS data extraction (via AT commands)
    - Network status
    """
    def __init__(self):
        super().__init__('lte_modem_node')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB2') # SIM7600 AT port is usually USB2
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('polling_rate', 1.0) # Hz
        self.declare_parameter('enable_gps', True)

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baudrate').value
        self.polling_rate = self.get_parameter('polling_rate').value
        self.gps_enabled = self.get_parameter('enable_gps').value

        # Publishers
        self.signal_pub = self.create_publisher(Int32, '~/rssi', 10)
        self.network_pub = self.create_publisher(String, '~/network_type', 10)
        self.sms_inbox_pub = self.create_publisher(String, '~/sms_received', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '~/gps', 10)

        # Subscribers
        self.create_subscription(String, '~/send_sms', self._send_sms_callback, 10)

        self.serial = None
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=1.0)
            self.get_logger().info(f"Connected to LTE Modem on {self.port}")
            self._init_modem()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to modem: {e}")

        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)

    def _send_at(self, command, back="OK", timeout=1):
        """Helper to send AT command and wait for response"""
        if not self.serial: return ""
        self.serial.write((command + "\r\n").encode())
        time.sleep(0.1)
        res = self.serial.read_all().decode(errors='ignore')
        return res

    def _init_modem(self):
        # Basic check
        self._send_at("AT")
        # Echo off
        self._send_at("ATE0")
        # SMS mode to Text
        self._send_at("AT+CMGF=1")
        # Enable GPS if requested
        if self.gps_enabled:
            self._send_at("AT+CGPS=1")
            self.get_logger().info("Modem GPS Enabled")

    def _send_sms_callback(self, msg):
        """Expects format: 'PHONE_NUMBER:MESSAGE'"""
        if not self.serial: return
        try:
            parts = msg.data.split(':', 1)
            if len(parts) < 2: return
            number, content = parts
            self.get_logger().info(f"Sending SMS to {number}...")
            self.serial.write(f'AT+CMGS virtual "{number}"\r'.encode())
            time.sleep(0.2)
            self.serial.write(f'{content}\x1A'.encode()) # \x1A is CTRL+Z
            self.get_logger().info("SMS sent command issued.")
        except Exception as e:
            self.get_logger().error(f"Error sending SMS: {e}")

    def timer_callback(self):
        if not self.serial: return

        # 1. Signal Strength (AT+CSQ)
        # Returns +CSQ: <rssi>,<ber>
        res = self._send_at("AT+CSQ")
        match = re.search(r"\+CSQ:\s+(\d+),", res)
        if match:
            rssi_val = int(match.group(1))
            # rssi 0 = -113dBm, 31 = -51dBm, 99 = unknown
            self.signal_pub.publish(Int32(data=rssi_val))

        # 2. Network Type (AT+CNSMOD?)
        # Returns +CNSMOD: <mode>,<stat>
        res = self._send_at("AT+CNSMOD?")
        if "4G" in res: self.network_pub.publish(String(data="4G/LTE"))
        elif "3G" in res: self.network_pub.publish(String(data="3G"))
        elif "2G" in res: self.network_pub.publish(String(data="2G/EDGE"))

        # 3. GPS Position (AT+CGPSINFO)
        if self.gps_enabled:
            res = self._send_at("AT+CGPSINFO")
            # Format: +CGPSINFO: [lat],[N/S],[lon],[E/W],[date],[time],[alt],[speed],[course]
            if "OK" in res and "," in res:
                self._parse_gps(res)

        # 4. Check for unread SMS (Optional, simplified)
        # This could be improved with URC notifications
        res = self._send_at('AT+CMGL="REC UNREAD"')
        if "+CMGL:" in res:
            self.sms_inbox_pub.publish(String(data=res))
            # Mark all as read
            self._send_at('AT+CMGD=1,1')

    def _parse_gps(self, raw):
        try:
            data = raw.replace("+CGPSINFO: ", "").split(",")
            if len(data) < 4 or not data[0]: return
            
            # Simple conversion from DDMM.MMMM to Decimal Degrees
            lat_raw = float(data[0])
            lat_deg = int(lat_raw / 100)
            lat_min = lat_raw - (lat_deg * 100)
            latitude = lat_deg + (lat_min / 60)
            if data[1] == 'S': latitude *= -1

            lon_raw = float(data[2])
            lon_deg = int(lon_raw / 100)
            lon_min = lon_raw - (lon_deg * 100)
            longitude = lon_deg + (lon_min / 60)
            if data[3] == 'W': longitude *= -1

            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps_link"
            msg.latitude = latitude
            msg.longitude = longitude
            if data[6]: msg.altitude = float(data[6])
            
            self.gps_pub.publish(msg)
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = LTEModemNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
