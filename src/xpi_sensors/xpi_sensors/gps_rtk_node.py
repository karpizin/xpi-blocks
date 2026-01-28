#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String, Int32
from pyubx2 import UBXReader, UBXMessage
import serial
import threading
import math

class GpsRtkNode(Node):
    """
    ROS2 Driver for RTK-capable GPS modules (e.g. u-blox ZED-F9P).
    Handles high-precision positioning, RTCM correction injection, and velocity.
    """
    def __init__(self):
        super().__init__('gps_rtk_node')

        # 1. Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('frequency', 5) # Desired update rate in Hz
        
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.freq = self.get_parameter('frequency').value

        # 2. Publishers
        self.fix_pub = self.create_publisher(NavSatFix, '~/fix', 10)
        self.vel_pub = self.create_publisher(TwistWithCovarianceStamped, '~/vel', 10)
        self.status_pub = self.create_publisher(Int32, '~/rtk_status', 10) # 0=No, 1=3D, 2=Float, 3=Fixed

        # 3. Subscribers (For RTCM corrections)
        self.create_subscription(String, '/rtk/corrections', self.correction_callback, 10)

        # 4. Serial Init
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.ubr = UBXReader(self.ser)
            self.get_logger().info(f"Connected to RTK GPS on {self.port} at {self.baud}")
            
            # Configure Rate
            self.configure_rate(self.freq)
            
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # 5. Reader Thread
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def configure_rate(self, rate_hz):
        """Sends UBX-CFG-RATE to set the update frequency."""
        if rate_hz <= 0: return
        meas_rate_ms = int(1000 / rate_hz)
        
        # CFG-RATE: msgClass=0x06, msgID=0x08
        msg = UBXMessage(
            "CFG",
            "CFG-RATE",
            measRate=meas_rate_ms,
            navRate=1,
            timeRef=1  # 1 = GPS Time
        )
        self.ser.write(msg.serialize())
        self.get_logger().info(f"Configured GPS update rate to {rate_hz}Hz ({meas_rate_ms}ms)")

    def correction_callback(self, msg):
        """Injects RTCM corrections from topic into the hardware serial port."""
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                data = bytes.fromhex(msg.data)
                self.ser.write(data)
            except ValueError:
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
                        self.publish_vel(parsed_data)
            except Exception as e:
                self.get_logger().warn(f"Read error: {e}")

    def publish_fix(self, data):
        """Converts UBX NAV-PVT data to ROS2 NavSatFix with Covariance."""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # UBX data is in 1e-7 degrees
        msg.latitude = data.lat * 1e-7
        msg.longitude = data.lon * 1e-7
        msg.altitude = float(data.hMSL * 1e-3) # Height above Mean Sea Level in meters

        # Covariance
        # hAcc and vAcc are in mm
        h_acc_m = data.hAcc / 1000.0
        v_acc_m = data.vAcc / 1000.0
        
        # Diagonal covariance matrix [E, N, U] -> [Lat, Lon, Alt] approximation
        msg.position_covariance = [
            h_acc_m**2, 0.0, 0.0,
            0.0, h_acc_m**2, 0.0,
            0.0, 0.0, v_acc_m**2
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        # RTK Status Mapping
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

    def publish_vel(self, data):
        """Publishes velocity and heading."""
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Velocity in ENU frame? No, typically body frame for Twist, but GPS gives global track.
        # gSpeed is ground speed (2D). headMot is heading of motion.
        
        speed_m_s = data.gSpeed / 1000.0
        heading_rad = math.radians(data.headMot * 1e-5) # headMot is 1e-5 deg
        
        # Convert polar (speed, heading) to Cartesian (x, y) relative to North?
        # Standard Twist message usually implies body frame velocity for robots, 
        # but for GPS 'vel' topic it often means global velocity vector.
        # Let's populate linear.x/y as East/North components.
        
        msg.twist.twist.linear.x = speed_m_s * math.sin(heading_rad) # East
        msg.twist.twist.linear.y = speed_m_s * math.cos(heading_rad) # North
        msg.twist.twist.linear.z = - (data.velD / 1000.0) # Down velocity to Up
        
        # Accuracy
        s_acc_m_s = data.sAcc / 1000.0
        msg.twist.covariance[0] = s_acc_m_s**2
        msg.twist.covariance[7] = s_acc_m_s**2
        msg.twist.covariance[14] = s_acc_m_s**2 # Approximation

        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsRtkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()