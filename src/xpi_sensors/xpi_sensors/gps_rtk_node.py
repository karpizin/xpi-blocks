#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Int32
import serial
import time

class GPSRTKNode(Node):
    def __init__(self):
        super().__init__('gps_rtk_node')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('frame_id', 'gps_link')
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Serial setup
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'GPS RTK: Connected to {port} at {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return

        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, '~/fix', 10)
        self.status_pub = self.create_publisher(Int32, '~/fix_status', 10)
        
        # Timer for polling
        self.create_timer(0.01, self.read_gps)

    def read_gps(self):
        if not self.ser.is_open: return
        
        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='replace').strip()
                if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                    self.parse_gga(line)
        except Exception as e:
            self.get_logger().error(f'Serial Error: {e}')

    def parse_gga(self, line):
        """
        GGA Format: $--GGA,time,lat,N,lon,E,fix_quality,num_sat,hdop,alt,M,sep,M,diff_age,diff_id*checksum
        Fix Quality: 0=No, 1=GPS, 2=DGPS, 4=Fixed RTK, 5=Float RTK
        """
        parts = line.split(',')
        if len(parts) < 10: return
        
        try:
            # 1. Status
            quality = int(parts[6])
            
            # Map NMEA quality to our simplified status
            # 4 -> 3 (Fixed), 5 -> 2 (Float), 1/2 -> 1 (GPS)
            mapped_status = 0
            if quality == 4: mapped_status = 3
            elif quality == 5: mapped_status = 2
            elif quality > 0: mapped_status = 1
            
            status_msg = Int32()
            status_msg.data = mapped_status
            self.status_pub.publish(status_msg)

            # 2. Coordinates (If Fix)
            if mapped_status > 0:
                msg = NavSatFix()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                
                # Parse Latitude (DDmm.mmmm)
                lat_raw = float(parts[2])
                lat_deg = int(lat_raw / 100)
                lat_min = lat_raw - (lat_deg * 100)
                msg.latitude = lat_deg + (lat_min / 60.0)
                if parts[3] == 'S': msg.latitude = -msg.latitude
                
                # Parse Longitude (DDDmm.mmmm)
                lon_raw = float(parts[4])
                lon_deg = int(lon_raw / 100)
                lon_min = lon_raw - (lon_deg * 100)
                msg.longitude = lon_deg + (lon_min / 60.0)
                if parts[5] == 'W': msg.longitude = -msg.longitude
                
                # Altitude
                msg.altitude = float(parts[9])
                
                # Accuracy Mapping (Simplified)
                if mapped_status == 3: # Fixed
                    msg.position_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
                elif mapped_status == 2: # Float
                    msg.position_covariance = [0.25, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.25]
                else:
                    msg.position_covariance = [4.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 4.0]
                
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                self.fix_pub.publish(msg)
                
        except (ValueError, IndexError):
            pass

def main(args=None):
    rclpy.init(args=args)
    node = GPSRTKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
