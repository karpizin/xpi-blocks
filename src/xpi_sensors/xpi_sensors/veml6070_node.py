import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from xpi_commons.i2c_helper import get_smbus
import time
import math

class VEML6070Node(Node):
    """
    ROS2 Node for the VEML6070 UV sensor.
    Uses two I2C addresses (0x38, 0x39) to read UVA intensity.
    Publishes raw counts and estimated UV risk level.
    """

    # VEML6070 I2C addresses
    ADDR_COMMAND = 0x38
    ADDR_DATA_MSB = 0x39
    ADDR_DATA_LSB = 0x38

    # Integration Times
    IT_1_2 = 0x00 # 1/2T
    IT_1   = 0x01 # 1T
    IT_2   = 0x02 # 2T
    IT_4   = 0x03 # 4T

    def __init__(self):
        super().__init__('veml6070_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('publish_rate', 1.0) # Hz
        self.declare_parameter('integration_time', self.IT_1)
        self.declare_parameter('mock_hardware', False)

        bus_id = self.get_parameter('i2c_bus').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.it_setting = self.get_parameter('integration_time').value
        mock_mode = self.get_parameter('mock_hardware').value

        # Init I2C
        self.bus = get_smbus(bus_id, mock=mock_mode)

        try:
            self.init_sensor()
            self.get_logger().info(f"VEML6070 initialized on bus {bus_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to communicate with VEML6070: {e}")
            if not mock_mode:
                self.get_logger().warn("Falling back to MOCK mode.")
                self.bus = get_smbus(bus_id, mock=True)

        # Publishers
        self.uv_pub = self.create_publisher(Int32, '~/uv_raw', 10)
        self.level_pub = self.create_publisher(String, '~/uv_index_level', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.mock_time = time.monotonic()

    def init_sensor(self):
        if self.bus.mock_mode: return
        # Command byte: bit 1 is Reserved (1), bit 0 is SD (Shutdown)
        # Integration time is in bits 2 and 3
        cmd = (self.it_setting << 2) | 0x02
        self.bus.write_byte(self.ADDR_COMMAND, cmd)
        time.sleep(0.2)

    def get_uv_risk_level(self, raw_value):
        # Based on Vishay's application note for 1T integration time
        # Values vary significantly based on integration time and external R_SET
        if raw_value < 560: return "Low"
        if raw_value < 1120: return "Moderate"
        if raw_value < 1494: return "High"
        if raw_value < 2054: return "Very High"
        return "Extreme"

    def timer_callback(self):
        try:
            if self.bus.mock_mode:
                t = time.monotonic() - self.mock_time
                uv_raw = int(800 + 400 * math.sin(t * math.pi / 20.0))
            else:
                # Read MSB and LSB
                msb = self.bus.read_byte(self.ADDR_DATA_MSB)
                lsb = self.bus.read_byte(self.ADDR_DATA_LSB)
                uv_raw = (msb << 8) | lsb

            # Publish
            msg_uv = Int32()
            msg_uv.data = uv_raw
            self.uv_pub.publish(msg_uv)

            msg_level = String()
            msg_level.data = self.get_uv_risk_level(uv_raw)
            self.level_pub.publish(msg_level)

            self.get_logger().debug(f"UV Raw: {uv_raw}, Level: {msg_level.data}")

        except Exception as e:
            self.get_logger().error(f"Error reading VEML6070: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VEML6070Node()
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
