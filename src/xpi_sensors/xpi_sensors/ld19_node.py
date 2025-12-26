#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import struct
import math

class LD19Node(Node):
    def __init__(self):
        super().__init__('ld19_node')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('frame_id', 'lidar_link')
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Serial setup
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'LD19: Connected to {port} at {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return

        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, '~/scan', 10)
        
        # Data accumulation
        self.points = []
        self.last_angle = 0
        
        # Timer for polling
        self.create_timer(0.001, self.read_and_parse)

    def read_and_parse(self):
        if not self.ser.is_open: return

        # Protocol LD19: Packet starts with 0x54, length 0x2C (44 bytes)
        while self.ser.in_waiting >= 44:
            if self.ser.read(1) == b'\x54':
                if self.ser.read(1) == b'\x2c':
                    packet = self.ser.read(42)
                    self.parse_packet(packet)

    def parse_packet(self, data):
        # Data format: 
        # [Speed(2)][StartAngle(2)][Data(12*3)][EndAngle(2)][Timestamp(2)][CRC(1)]
        # Total 44 bytes (2 read already)
        speed = struct.unpack('<H', data[0:2])[0]
        start_angle = struct.unpack('<H', data[2:4])[0] / 100.0
        
        # 12 points of (distance, intensity)
        batch_points = []
        for i in range(12):
            dist = struct.unpack('<H', data[4+i*3 : 6+i*3])[0] / 1000.0 # to meters
            intensity = data[6+i*3]
            batch_points.append((dist, intensity))
            
        end_angle = struct.unpack('<H', data[40:42])[0] / 100.0
        
        # Calculate angle step
        if end_angle < start_angle:
            diff = (end_angle + 360) - start_angle
        else:
            diff = end_angle - start_angle
        step = diff / 11.0
        
        for i, (dist, intens) in enumerate(batch_points):
            angle = (start_angle + i * step) % 360
            self.points.append((angle, dist, intens))
            
            # If we completed a full circle (0-360)
            if angle < self.last_angle:
                self.publish_scan()
                self.points = []
            self.last_angle = angle

    def publish_scan(self):
        if not self.points: return
        
        # Sort points by angle
        self.points.sort(key=lambda x: x[0])
        
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = (2 * math.pi) / len(self.points)
        msg.time_increment = 0.0 # Assumed instantaneous for now
        msg.scan_time = 0.1 # 10Hz typical
        msg.range_min = 0.02
        msg.range_max = 12.0
        
        msg.ranges = [p[1] for p in self.points]
        msg.intensities = [float(p[2]) for p in self.points]
        
        self.scan_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LD19Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
