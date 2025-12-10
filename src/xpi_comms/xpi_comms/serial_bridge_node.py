#!/usr/bin/env python3
import time
import threading
import serial
import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterDescriptor, ParameterType

from std_msgs.msg import String, UInt8MultiArray

class SerialBridgeNode(Node):
    """
    A generic Serial Bridge Node that supports Text (Line-based) and Binary modes.
    """
    def __init__(self):
        super().__init__('serial_bridge')

        # 1. Parameters
        self.declare_parameter('port', '/dev/ttyUSB0', 
            ParameterDescriptor(description='Serial port device path'))
        self.declare_parameter('baudrate', 115200, 
            ParameterDescriptor(description='Baud rate for serial communication'))
        self.declare_parameter('timeout', 1.0, 
            ParameterDescriptor(description='Read timeout in seconds'))
        self.declare_parameter('mode', 'text', 
            ParameterDescriptor(description='Operation mode: "text" (lines) or "binary" (raw bytes)'))

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.mode = self.get_parameter('mode').value.lower()

        if self.mode not in ['text', 'binary']:
            self.get_logger().error(f"Invalid mode '{self.mode}'. Defaulting to 'text'.")
            self.mode = 'text'

        # 2. Serial Connection
        self.serial_conn = None
        self._connect_serial()

        # 3. ROS Interfaces
        # TX (ROS -> Serial)
        if self.mode == 'text':
            self.sub_tx = self.create_subscription(
                String, '~/tx', self._tx_text_callback, 10)
        else:
            self.sub_tx = self.create_subscription(
                UInt8MultiArray, '~/tx', self._tx_binary_callback, 10)

        # RX (Serial -> ROS)
        if self.mode == 'text':
            self.pub_rx = self.create_publisher(String, '~/rx', 10)
        else:
            self.pub_rx = self.create_publisher(UInt8MultiArray, '~/rx', 10)

        # 4. Read Thread
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info(f"Serial Bridge started on {self.port} @ {self.baudrate} ({self.mode} mode)")

    def _connect_serial(self):
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            # We don't exit, we might retry in the loop or let the user fix it and restart

    def _read_loop(self):
        """
        Continuously reads from serial port and publishes to ROS.
        """
        while self.running and rclpy.ok():
            if self.serial_conn is None or not self.serial_conn.is_open:
                time.sleep(1.0) # Wait before retrying
                # Optional: Try to reconnect?
                # For now, simplistic approach: if it failed at start, it stays failed. 
                # Improving robustness:
                if self.serial_conn is None:
                     try:
                        self.serial_conn = serial.Serial(
                            port=self.port,
                            baudrate=self.baudrate,
                            timeout=self.timeout
                        )
                        self.get_logger().info(f"Reconnected to {self.port}")
                     except Exception:
                        continue
                continue

            try:
                if self.mode == 'text':
                    # Read line
                    line = self.serial_conn.readline()
                    if line:
                        try:
                            decoded = line.decode('utf-8').strip()
                            if decoded: # Don't publish empty lines
                                msg = String()
                                msg.data = decoded
                                self.pub_rx.publish(msg)
                        except UnicodeDecodeError:
                            self.get_logger().warn("Received non-UTF-8 data in text mode")
                
                elif self.mode == 'binary':
                    # Read available bytes
                    if self.serial_conn.in_waiting > 0:
                        data = self.serial_conn.read(self.serial_conn.in_waiting)
                        if data:
                            msg = UInt8MultiArray()
                            msg.data = list(data)
                            self.pub_rx.publish(msg)
                    else:
                        time.sleep(0.01) # Avoid busy loop

            except Exception as e:
                self.get_logger().error(f"Error in read loop: {e}")
                time.sleep(1.0)

    def _tx_text_callback(self, msg):
        """
        Handle text sending. Appends newline if missing.
        """
        if self.serial_conn and self.serial_conn.is_open:
            data = msg.data
            if not data.endswith('\n'):
                data += '\n'
            try:
                self.serial_conn.write(data.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Failed to write text: {e}")

    def _tx_binary_callback(self, msg):
        """
        Handle binary sending.
        """
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(bytes(msg.data))
            except Exception as e:
                self.get_logger().error(f"Failed to write binary: {e}")

    def destroy_node(self):
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
