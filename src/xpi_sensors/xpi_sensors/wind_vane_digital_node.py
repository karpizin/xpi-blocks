import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from xpi_commons.i2c_helper import get_smbus
import time
import math

class WindVaneDigitalNode(Node):
    """
    ROS2 Node for a Digital Wind Vane using AS5600 magnetic encoder.
    Reads absolute angle via I2C and publishes direction in degrees and cardinal points.
    """

    AS5600_ADDR = 0x36
    REG_STATUS = 0x0B
    REG_RAW_ANGLE = 0x0C

    CARDINALS = [
        (11.25, "N"), (33.75, "NNE"), (56.25, "NE"), (78.75, "ENE"),
        (101.25, "E"), (123.75, "ESE"), (146.25, "SE"), (168.75, "SSE"),
        (191.25, "S"), (213.75, "SSW"), (236.25, "SW"), (258.75, "WSW"),
        (281.25, "W"), (303.75, "WNW"), (326.25, "NW"), (348.75, "NNW"),
        (360.0, "N")
    ]

    def __init__(self):
        super().__init__('wind_vane_digital_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('publish_rate', 5.0) # Hz
        self.declare_parameter('offset_degrees', 0.0) # To align North
        self.declare_parameter('mock_hardware', False)

        bus_id = self.get_parameter('i2c_bus').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.offset = self.get_parameter('offset_degrees').value
        mock_mode = self.get_parameter('mock_hardware').value

        # Init I2C
        self.bus = get_smbus(bus_id, mock=mock_mode)

        try:
            self.check_status()
            self.get_logger().info(f"AS5600 Digital Wind Vane initialized on bus {bus_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to communicate with AS5600: {e}")
            if not mock_mode:
                self.get_logger().warn("Falling back to MOCK mode.")
                self.bus = get_smbus(bus_id, mock=True)

        # Publishers
        self.dir_pub = self.create_publisher(Float32, '~/direction', 10)
        self.card_pub = self.create_publisher(String, '~/cardinal', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.mock_start_time = time.monotonic()

    def check_status(self):
        if self.bus.mock_mode: return
        status = self.bus.read_byte_data(self.AS5600_ADDR, self.REG_STATUS)
        # Bit 5: Magnet detected, Bit 4: Too weak, Bit 3: Too strong
        if not (status & 0x20):
            self.get_logger().warn("AS5600: Magnet NOT detected!")

    def get_cardinal(self, degrees):
        for limit, name in self.CARDINALS:
            if degrees < limit:
                return name
        return "N"

    def timer_callback(self):
        try:
            if self.bus.mock_mode:
                # Simulate wind rotation
                t = time.monotonic() - self.mock_start_time
                angle = (t * 10.0) % 360.0
                status_str = "Magnet OK (Mock)"
            else:
                # Read 12-bit angle (0-4095)
                high = self.bus.read_byte_data(self.AS5600_ADDR, self.REG_RAW_ANGLE)
                low = self.bus.read_byte_data(self.AS5600_ADDR, self.REG_RAW_ANGLE + 1)
                raw_angle = (high << 8) | low
                angle = (raw_angle / 4096.0) * 360.0
                
                # Apply offset and normalize
                angle = (angle + self.offset) % 360.0
                
                # Check status
                status = self.bus.read_byte_data(self.AS5600_ADDR, self.REG_STATUS)
                if status & 0x20: status_str = "Magnet OK"
                elif status & 0x10: status_str = "Magnet Too Weak"
                elif status & 0x08: status_str = "Magnet Too Strong"
                else: status_str = "No Magnet"

            # Publish
            msg_dir = Float32()
            msg_dir.data = float(angle)
            self.dir_pub.publish(msg_dir)

            msg_card = String()
            msg_card.data = self.get_cardinal(angle)
            self.card_pub.publish(msg_card)

            msg_stat = String()
            msg_status = String()
            msg_status.data = status_str
            self.status_pub.publish(msg_status)

            self.get_logger().debug(f"Angle: {angle:.2f} ({msg_card.data}) - {status_str}")

        except Exception as e:
            self.get_logger().error(f"Error reading AS5600: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WindVaneDigitalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
