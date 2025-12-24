import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
from gpiozero import OutputDevice, Device
from gpiozero.pins.mock import MockFactory

class HC12Node(Node):
    def __init__(self):
        super().__init__('hc12_node')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyS0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('set_pin', 17)
        self.declare_parameter('use_mock', False)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        set_pin_num = self.get_parameter('set_pin').value
        use_mock = self.get_parameter('use_mock').value

        if use_mock:
            Device.pin_factory = MockFactory()
            self.get_logger().info("Using Mock GPIO for SET pin")

        # Hardware Setup
        try:
            # We don't open serial here if it might fail, to allow node to start
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to HC-12 on {port} at {baudrate} bps")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            # In a real robot, we might want to retry or keep node alive
            raise e

        # SET pin is Active LOW. initial_value=False means HIGH (Data Mode)
        self.set_pin = OutputDevice(set_pin_num, active_high=False, initial_value=False)
        
        # Publishers & Subscribers
        self.rx_pub = self.create_publisher(String, '~/rx', 10)
        self.at_resp_pub = self.create_publisher(String, '~/at_response', 10)
        
        self.tx_sub = self.create_subscription(String, '~/tx', self.tx_callback, 10)
        self.at_sub = self.create_subscription(String, '~/cmd_at', self.at_callback, 10)
        
        # Timer for reading data mode
        self.timer = self.create_timer(0.05, self.read_serial)

        self.get_logger().info("HC-12 Node initialized. Use ~/cmd_at to send AT commands.")

    def tx_callback(self, msg):
        """Sends data over the air in Transparent Mode."""
        try:
            self.ser.write(msg.data.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"TX Error: {e}")

    def at_callback(self, msg):
        """Handle AT command request."""
        cmd = msg.data.strip()
        if not cmd.upper().startswith("AT"):
            self.get_logger().warn(f"Invalid AT command: {cmd}")
            return
        
        response = self.send_at_command(cmd)
        
        resp_msg = String()
        resp_msg.data = response
        self.at_resp_pub.publish(resp_msg)

    def read_serial(self):
        """Reads data from the module and publishes it."""
        if self.ser.in_waiting > 0:
            try:
                data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                if data:
                    msg = String()
                    msg.data = data
                    self.rx_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"RX Error: {e}")

    def send_at_command(self, cmd):
        """Toggles SET pin, sends command, returns response."""
        self.get_logger().info(f"Sending AT Command: {cmd}")
        self.set_pin.on() # Pulls LOW (Active LOW)
        time.sleep(0.1) # Spec says >40ms
        
        self.ser.write(cmd.encode('utf-8'))
        time.sleep(0.5) # Wait for response
        
        response = ""
        if self.ser.in_waiting > 0:
            response = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
        
        self.set_pin.off() # Back to HIGH (Data mode)
        time.sleep(0.1) # Wait for module to resume data mode
        
        clean_resp = response.strip()
        self.get_logger().info(f"Response: {clean_resp}")
        return clean_resp

def main(args=None):
    rclpy.init(args=args)
    node = HC12Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
