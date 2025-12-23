#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import serial
import struct
import time

class PMS5003Node(Node):
    """
    ROS2 Node for Plantower PMS5003 / PMS7003 Particulate Matter Sensors.
    Reads PM1.0, PM2.5, and PM10 via UART (Serial).
    """
    def __init__(self):
        super().__init__('pms5003_node')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('polling_rate', 1.0)
        self.declare_parameter('frame_id', 'pms5003_link')
        self.declare_parameter('mock_hardware', False)

        self.port_name = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.polling_rate = self.get_parameter('polling_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        # Publishers
        # Using Atmospheric environment values (ug/m3)
        self.pm1_pub = self.create_publisher(Float32, '~/pm1_0', 10)
        self.pm25_pub = self.create_publisher(Float32, '~/pm2_5', 10)
        self.pm10_pub = self.create_publisher(Float32, '~/pm10', 10)
        
        # Combined array for convenience: [PM1.0, PM2.5, PM10]
        self.all_pub = self.create_publisher(Float32MultiArray, '~/data', 10)

        self.serial = None

        if not self.mock_mode:
            try:
                self.serial = serial.Serial(
                    port=self.port_name,
                    baudrate=self.baudrate,
                    timeout=2.0
                )
                self.get_logger().info(f"Opened serial port {self.port_name} for PMS5003.")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port {self.port_name}: {e}. Falling back to MOCK.")
                self.mock_mode = True
        
        if self.mock_mode:
            self.get_logger().warn("Running PMS5003 in MOCK mode.")

        self.timer = self.create_timer(1.0 / self.polling_rate, self.timer_callback)

    def _verify_checksum(self, data):
        # Last 2 bytes are checksum (Big Endian)
        expected_checksum = (data[30] << 8) | data[31]
        # Sum of first 30 bytes
        calculated_checksum = sum(data[0:30])
        return expected_checksum == calculated_checksum

    def timer_callback(self):
        if self.mock_mode:
            # Generate mock data
            import math
            t = time.monotonic()
            pm1 = 5.0 + 2.0 * math.sin(t / 5.0)
            pm25 = 12.0 + 5.0 * math.sin(t / 7.0)
            pm10 = 20.0 + 8.0 * math.sin(t / 9.0)
            self._publish(pm1, pm25, pm10)
            return

        if self.serial and self.serial.is_open:
            try:
                # PMS5003 sends 32-byte frames starting with 0x42 0x4D
                # Reset buffer if too full to avoid reading old data
                if self.serial.in_waiting > 128:
                    self.serial.reset_input_buffer()
                
                # We need at least 32 bytes
                if self.serial.in_waiting < 32:
                    return # Wait for more data

                # Read standard 32 bytes
                # To sync, we might need to read byte by byte until we find header, 
                # but simplified approach works often if stream is continuous.
                # Robust approach:
                
                header = self.serial.read(1)
                if header != b'\x42':
                    return # Not start of frame
                
                header2 = self.serial.read(1)
                if header2 != b'\x4D':
                    return # Not start of frame

                # Read remaining 30 bytes
                data_payload = self.serial.read(30)
                if len(data_payload) != 30:
                    return

                full_frame = b'\x42\x4D' + data_payload
                
                if not self._verify_checksum(full_frame):
                    self.get_logger().debug("PMS5003 Checksum mismatch")
                    return

                # Parse data (Big Endian standard)
                # Bytes 10-11: PM1.0 (Atmosphere)
                # Bytes 12-13: PM2.5 (Atmosphere)
                # Bytes 14-15: PM10  (Atmosphere)
                
                # struct format: > = Big Endian, H = unsigned short (2 bytes)
                # The data_payload starts at index 2 of full frame (Frame Length high byte)
                # Unpacking whole payload for clarity:
                # Frame Len(2), PM1_Std(2), PM25_Std(2), PM10_Std(2), PM1_Atm(2), PM25_Atm(2), PM10_Atm(2)
                
                values = struct.unpack('>HHHHHHHHHHHHH', data_payload[0:26])
                
                # values[3] -> PM1.0 Atm
                # values[4] -> PM2.5 Atm
                # values[5] -> PM10 Atm
                
                pm1_atm = float(values[3])
                pm25_atm = float(values[4])
                pm10_atm = float(values[5])

                self._publish(pm1_atm, pm25_atm, pm10_atm)

            except Exception as e:
                self.get_logger().error(f"Error reading serial: {e}")

    def _publish(self, pm1, pm25, pm10):
        # Publish individual topics
        self.pm1_pub.publish(Float32(data=pm1))
        self.pm25_pub.publish(Float32(data=pm25))
        self.pm10_pub.publish(Float32(data=pm10))
        
        # Publish array
        msg = Float32MultiArray()
        msg.data = [pm1, pm25, pm10]
        self.all_pub.publish(msg)
        
        self.get_logger().debug(f"PM1.0: {pm1}, PM2.5: {pm25}, PM10: {pm10}")

    def destroy_node(self):
        if self.serial:
            self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PMS5003Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
