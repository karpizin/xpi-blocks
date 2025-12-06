import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import struct
import time
import os

class CrsfReceiverNode(Node):
    """
    ROS2 Node for receiving CRSF (Crossfire/ELRS) data via UART and publishing it as sensor_msgs/Joy.
    """

    def __init__(self):
        super().__init__('crsf_receiver_node')

        # 1. Declare Parameters
        self.declare_parameter('uart_port', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 420000) # CRSF standard baud rate
        self.declare_parameter('publish_rate', 100.0) # Hz, CRSF is faster than SBUS
        self.declare_parameter('mock_hardware', False) # For testing without real UART

        # 2. Read Parameters
        self.uart_port = self.get_parameter('uart_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        self.ser = None
        if not self.mock_mode:
            try:
                # Open serial port
                self.ser = serial.Serial(
                    port=self.uart_port,
                    baudrate=self.baud_rate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE, # CRSF uses NONE parity
                    stopbits=serial.STOPBITS_ONE, # CRSF uses 1 stop bit
                    timeout=0.001 # Non-blocking read
                )
                self.get_logger().info(f'CRSF: Opened UART port {self.uart_port} at {self.baud_rate} baud.')
            except serial.SerialException as e:
                self.get_logger().error(f'CRSF: Failed to open serial port {self.uart_port}: {e}')
                self.mock_mode = True # Fallback to mock if real port fails

        if self.mock_mode:
            self.get_logger().warn('CRSF: Running in MOCK mode. No real CRSF data will be read.')

        # 3. Publisher
        self.joy_publisher = self.create_publisher(Joy, '~/joy', 10)
        
        # 4. Timer for publishing and reading
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # CRSF frame constants
        self.CRSF_SYNC_BYTE = 0xC8
        self.CRSF_MAX_PACKET_LEN = 64 # Max length includes sync byte and length byte
        self.CRSF_MIN_PACKET_LEN = 4  # Sync + len + type + CRC

        # CRSF Frame types
        self.CRSF_FRAMETYPE_RC_CHANNELS_PACKET = 0x16

        self.rx_buffer = bytearray()
        self.last_frame_time = time.monotonic()

    def timer_callback(self):
        if self.mock_mode:
            self.publish_mock_data()
            return
        
        if self.ser and self.ser.is_open:
            try:
                self.rx_buffer.extend(self.ser.read(self.ser.in_waiting))
                
                while len(self.rx_buffer) >= self.CRSF_MIN_PACKET_LEN:
                    # Find sync byte
                    if self.rx_buffer[0] != self.CRSF_SYNC_BYTE:
                        # self.get_logger().debug(f"CRSF: Skipping byte 0x{self.rx_buffer[0]:02X}, not sync byte.")
                        self.rx_buffer.pop(0) # Remove invalid byte
                        continue
                    
                    # Read length byte
                    packet_len = self.rx_buffer[1] # Length includes type and payload, but not sync and length byte themselves
                    full_packet_len = packet_len + 2 # Add sync and length bytes
                    
                    if full_packet_len > self.CRSF_MAX_PACKET_LEN or packet_len < 2:
                        self.get_logger().warn(f"CRSF: Invalid packet length {packet_len}. Resyncing.")
                        self.rx_buffer.pop(0) # Remove sync byte and try again
                        continue

                    if len(self.rx_buffer) < full_packet_len:
                        # Not enough data for full packet
                        break 
                    
                    # Extract packet
                    packet = self.rx_buffer[0:full_packet_len]
                    self.rx_buffer = self.rx_buffer[full_packet_len:] # Remove from buffer

                    # Check CRC (simple 8-bit XOR checksum)
                    expected_crc = packet[full_packet_len - 1]
                    calculated_crc = self._crc8_poly(packet[2:full_packet_len-1]) # Exclude sync, length, and actual CRC
                    
                    if expected_crc != calculated_crc:
                        self.get_logger().warn(f"CRSF: CRC mismatch. Expected 0x{expected_crc:02X}, got 0x{calculated_crc:02X}. Packet type 0x{packet[2]:02X}.")
                        continue
                    
                    # Process valid packet
                    packet_type = packet[2]
                    payload = packet[3:full_packet_len-1] # Exclude sync, length, type, CRC

                    if packet_type == self.CRSF_FRAMETYPE_RC_CHANNELS_PACKET:
                        self.process_rc_channels_packet(payload)
                    # else:
                        # self.get_logger().debug(f"CRSF: Unhandled packet type 0x{packet_type:02X} with length {packet_len}")
                        
            except serial.SerialException as e:
                self.get_logger().error(f'CRSF: Serial read error: {e}')
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.mock_mode = True # Fallback to mock

    def process_rc_channels_packet(self, payload):
        """
        Parses RC Channels Payload (11-bit channels, 16 channels in 22 bytes)
        Payload is 22 bytes.
        """
        if len(payload) != 22:
            self.get_logger().warn(f"CRSF: RC Channels packet has unexpected payload length: {len(payload)}")
            return

        channels = [0] * 16 # CRSF has 16 channels
        
        # CRSF 11-bit channel decoding (similar to SBUS, but specific byte order)
        # Each channel value is 11 bits.
        # This is based on Betaflight CRSF decoding.
        
        # Bytes 0-1
        channels[0]  = (payload[0] | payload[1] << 8) & 0x07FF
        # Bytes 1-2
        channels[1]  = (payload[1] >> 3 | payload[2] << 5) & 0x07FF
        # Bytes 2-4
        channels[2]  = (payload[2] >> 6 | payload[3] << 2 | payload[4] << 10) & 0x07FF
        # Bytes 4-5
        channels[3]  = (payload[4] >> 1 | payload[5] << 7) & 0x07FF
        # Bytes 5-6
        channels[4]  = (payload[5] >> 4 | payload[6] << 4) & 0x07FF
        # Bytes 6-8
        channels[5]  = (payload[6] >> 7 | payload[7] << 1 | payload[8] << 9) & 0x07FF
        # Bytes 8-9
        channels[6]  = (payload[8] >> 2 | payload[9] << 6) & 0x07FF
        # Bytes 9-10
        channels[7]  = (payload[9] >> 5 | payload[10] << 3) & 0x07FF
        
        # And repeat for channels 8-15 from bytes 11-21
        channels[8]  = (payload[11] | payload[12] << 8) & 0x07FF
        channels[9]  = (payload[12] >> 3 | payload[13] << 5) & 0x07FF
        channels[10] = (payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF
        channels[11] = (payload[15] >> 1 | payload[16] << 7) & 0x07FF
        channels[12] = (payload[16] >> 4 | payload[17] << 4) & 0x07FF
        channels[13] = (payload[17] >> 7 | payload[18] << 1 | payload[19] << 9) & 0x07FF
        channels[14] = (payload[19] >> 2 | payload[20] << 6) & 0x07FF
        channels[15] = (payload[20] >> 5 | payload[21] << 3) & 0x07FF
        
        # CRSF raw channel range is 172 (min) to 1811 (max), center 992
        # Normalize to -1.0 to 1.0
        norm_channels = []
        for ch_val in channels:
            # Map 172-992-1811 to -1.0-0.0-1.0
            norm_channels.append((ch_val - 992.0) / 819.0) # 1811-992 = 819

        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = norm_channels
        joy_msg.buttons = [] # CRSF typically doesn't send failsafe/frame_lost as explicit flags in RC_CHANNELS
        
        self.joy_publisher.publish(joy_msg)
        self.last_frame_time = time.monotonic()

    def _crc8_poly(self, data):
        """
        CRSF uses a polynomial 0xD5 (11010101) for its 8-bit XOR checksum.
        This is a common CRSF CRC-8 implementation.
        """
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0xD5
                else:
                    crc <<= 1
        return crc & 0xFF # Ensure it's 8 bits

    def publish_mock_data(self):
        """ Publishes dummy Joy messages for mock mode. """
        current_time = time.monotonic()
        if (current_time - self.last_frame_time) < (1.0 / self.publish_rate):
            return # Don't publish too fast
            
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        # Simulate moving stick
        t = current_time % 4.0
        joy_msg.axes = [
            math.sin(t * math.pi / 2.0),  # Channel 0 (left stick X)
            math.cos(t * math.pi / 2.0),  # Channel 1 (left stick Y)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, # Other channels
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ]
        joy_msg.buttons = []
        self.joy_publisher.publish(joy_msg)
        self.last_frame_time = current_time


    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('CRSF: Closed UART port.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CrsfReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
