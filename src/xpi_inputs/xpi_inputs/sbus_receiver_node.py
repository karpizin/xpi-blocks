import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import struct
import time
import os

class SbusReceiverNode(Node):
    """
    ROS2 Node for receiving SBUS data via UART and publishing it as sensor_msgs/Joy.
    """

    def __init__(self):
        super().__init__('sbus_receiver_node')

        # 1. Declare Parameters
        self.declare_parameter('uart_port', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 100000) # SBUS standard baud rate
        self.declare_parameter('publish_rate', 50.0) # Hz, SBUS is ~10ms frame rate
        self.declare_parameter('mock_hardware', False) # For testing without real UART
        self.declare_parameter('invert_sbus', False) # Some SBUS inverters exist

        # 2. Read Parameters
        self.uart_port = self.get_parameter('uart_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.mock_mode = self.get_parameter('mock_hardware').value
        self.invert_sbus = self.get_parameter('invert_sbus').value

        self.ser = None
        if not self.mock_mode:
            try:
                # Open serial port
                self.ser = serial.Serial(
                    port=self.uart_port,
                    baudrate=self.baud_rate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_EVEN, # SBUS uses EVEN parity
                    stopbits=serial.STOPBITS_TWO, # SBUS uses 2 stop bits
                    timeout=0.01 # Non-blocking read
                )
                self.get_logger().info(f'SBUS: Opened UART port {self.uart_port} at {self.baud_rate} baud.')
            except serial.SerialException as e:
                self.get_logger().error(f'SBUS: Failed to open serial port {self.uart_port}: {e}')
                self.mock_mode = True # Fallback to mock if real port fails

        if self.mock_mode:
            self.get_logger().warn('SBUS: Running in MOCK mode. No real SBUS data will be read.')

        # 3. Publisher
        self.joy_publisher = self.create_publisher(Joy, '~/joy', 10)
        
        # 4. Timer for publishing and reading
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # SBUS frame constants
        self.SBUS_STARTBYTE = 0x0F
        self.SBUS_ENDBYTE = 0x00 # or 0x04 for some extended frames
        self.SBUS_FRAME_SIZE = 25 # 1 start byte + 22 data bytes + 1 end byte + 1 failsafe byte (total 25)
        self.SBUS_NUM_CHANNELS = 16

        self.sbus_frame = bytearray(self.SBUS_FRAME_SIZE)
        self.frame_index = 0
        self.last_frame_time = time.monotonic()

    def timer_callback(self):
        if self.mock_mode:
            self.publish_mock_data()
            return
        
        if self.ser and self.ser.is_open:
            try:
                # Read all available bytes
                while self.ser.in_waiting > 0:
                    byte = self.ser.read(1)
                    if not byte:
                        continue
                    
                    byte = ord(byte) # Convert byte to int
                    
                    # Look for start byte
                    if self.frame_index == 0:
                        if byte == self.SBUS_STARTBYTE:
                            self.sbus_frame[self.frame_index] = byte
                            self.frame_index += 1
                        else:
                            self.get_logger().debug(f"SBUS: Skipping byte 0x{byte:02X}, not start byte.")
                        continue
                    
                    self.sbus_frame[self.frame_index] = byte
                    self.frame_index += 1

                    if self.frame_index == self.SBUS_FRAME_SIZE:
                        # Full frame received
                        self.frame_index = 0 # Reset for next frame
                        if self.sbus_frame[self.SBUS_FRAME_SIZE - 1] == self.SBUS_ENDBYTE:
                            self.process_sbus_frame(self.sbus_frame)
                        else:
                            self.get_logger().warn(f"SBUS: Invalid end byte 0x{self.sbus_frame[self.SBUS_FRAME_SIZE - 1]:02X}")
                            # Try to resync by finding the next start byte in the buffer
                            try:
                                next_start = self.sbus_frame.index(self.SBUS_STARTBYTE, 1) # search from 2nd byte
                                self.sbus_frame = self.sbus_frame[next_start:] + bytearray(next_start)
                                self.frame_index = len(self.sbus_frame) - next_start
                                self.get_logger().debug(f"SBUS: Resynced. New index {self.frame_index}")
                            except ValueError:
                                pass # No start byte found, will reset next time
                                
            except serial.SerialException as e:
                self.get_logger().error(f'SBUS: Serial read error: {e}')
                self.ser.close()
                self.mock_mode = True # Fallback to mock

    def process_sbus_frame(self, frame):
        # SBUS data is 22 bytes after start byte, 16 channels, 11 bits each
        # Bytes 1-22 are channel data
        
        # Invert SBUS if necessary (some older receivers or inverters)
        data = frame[1:23] # Extract 22 data bytes
        if self.invert_sbus:
            data = bytearray([~b & 0xFF for b in data])

        channels = [0] * self.SBUS_NUM_CHANNELS
        
        # Decode 11-bit channel data from 22 bytes
        channels[0]  = (data[0] | data[1] << 8) & 0x07FF
        channels[1]  = (data[1] >> 3 | data[2] << 5) & 0x07FF
        channels[2]  = (data[2] >> 6 | data[3] << 2 | data[4] << 10) & 0x07FF
        channels[3]  = (data[4] >> 1 | data[5] << 7) & 0x07FF
        channels[4]  = (data[5] >> 4 | data[6] << 4) & 0x07FF
        channels[5]  = (data[6] >> 7 | data[7] << 1 | data[8] << 9) & 0x07FF
        channels[6]  = (data[8] >> 2 | data[9] << 6) & 0x07FF
        channels[7]  = (data[9] >> 5 | data[10] << 3) & 0x07FF
        channels[8]  = (data[11] | data[12] << 8) & 0x07FF
        channels[9]  = (data[12] >> 3 | data[13] << 5) & 0x07FF
        channels[10] = (data[13] >> 6 | data[14] << 2 | data[15] << 10) & 0x07FF
        channels[11] = (data[15] >> 1 | data[16] << 7) & 0x07FF
        channels[12] = (data[16] >> 4 | data[17] << 4) & 0x07FF
        channels[13] = (data[17] >> 7 | data[18] << 1 | data[19] << 9) & 0x07FF
        channels[14] = (data[19] >> 2 | data[20] << 6) & 0x07FF
        channels[15] = (data[20] >> 5 | data[21] << 3) & 0x07FF

        # Failsafe and lost frames flags (last byte of data section)
        flags = data[22]
        failsafe = bool(flags & (1 << 3))
        frame_lost = bool(flags & (1 << 2))

        # Normalize channels to -1.0 to 1.0 range (SBUS raw 172-1811)
        # 172 -> -1.0, 992 -> 0.0, 1811 -> 1.0 (approx)
        # Based on common values, 992 is center, range ~ +/- 820
        norm_channels = []
        for ch_val in channels:
            norm_channels.append((ch_val - 992.0) / 820.0) # Adjust based on actual receiver calibration
        
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = norm_channels
        joy_msg.buttons = [1 if failsafe else 0, 1 if frame_lost else 0] # Use buttons for status
        
        self.joy_publisher.publish(joy_msg)
        self.last_frame_time = time.monotonic()


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
        joy_msg.buttons = [0, 0] # No failsafe/lost
        self.joy_publisher.publish(joy_msg)
        self.last_frame_time = current_time


    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('SBUS: Closed UART port.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SbusReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
