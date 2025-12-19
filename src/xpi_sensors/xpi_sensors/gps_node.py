#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyS0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'gps_link')
        
        self.port_name = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, '~/fix', 10)
        self.time_pub = self.create_publisher(TimeReference, '~/time_reference', 10)

        # Serial Setup
        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=1.0)
            self.get_logger().info(f"GPS Serial port opened: {self.port_name} @ {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"Failed to open GPS serial port: {e}")
            self.ser = None

        # Timer for polling serial (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if not self.ser:
            return

        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='replace').strip()
                if line.startswith('$'):
                    try:
                        msg = pynmea2.parse(line)
                        self.handle_nmea_msg(msg)
                    except pynmea2.ParseError:
                        continue
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def handle_nmea_msg(self, msg):
        current_time = self.get_clock().now().to_msg()

        # GGA contains Altitude and Fix Quality
        if isinstance(msg, pynmea2.types.talker.GGA):
            fix_msg = NavSatFix()
            fix_msg.header.stamp = current_time
            fix_msg.header.frame_id = self.frame_id
            
            # Status
            if msg.gps_qual > 0:
                fix_msg.status.status = NavSatStatus.STATUS_FIX
            else:
                fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
            
            fix_msg.status.service = NavSatStatus.SERVICE_GPS
            
            # Coordinates
            fix_msg.latitude = msg.latitude
            fix_msg.longitude = msg.longitude
            fix_msg.altitude = float(msg.altitude) if msg.altitude else 0.0
            
            # Covariance (Simplified: diagonal with HDOP)
            hdop = float(msg.horizontal_dil) if msg.horizontal_dil else 100.0
            fix_msg.position_covariance = [hdop**2, 0.0, 0.0, 
                                           0.0, hdop**2, 0.0, 
                                           0.0, 0.0, (2*hdop)**2]
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            
            self.fix_pub.publish(fix_msg)

        # RMC contains time and status
        elif isinstance(msg, pynmea2.types.talker.RMC):
            if msg.status == 'A': # Active fix
                time_msg = TimeReference()
                time_msg.header.stamp = current_time
                time_msg.header.frame_id = self.frame_id
                # Note: msg.datetime might be used for true GPS time
                self.time_pub.publish(time_msg)

    def destroy_node(self):
        if self.ser:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
