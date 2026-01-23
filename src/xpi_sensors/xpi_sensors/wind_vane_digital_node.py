import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from xpi_commons.i2c_helper import get_smbus
import time

class WindVaneDigitalNode(Node):
# ... (intermediate code)
    def check_status(self):
        if self.bus.mock_mode:
            return
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
                if status & 0x20:
                    status_str = "Magnet OK"
                elif status & 0x10:
                    status_str = "Magnet Too Weak"
                elif status & 0x08:
                    status_str = "Magnet Too Strong"
                else:
                    status_str = "No Magnet"

            # Publish
            msg_dir = Float32()
            msg_dir.data = float(angle)
            self.dir_pub.publish(msg_dir)

            msg_card = String()
            msg_card.data = self.get_cardinal(angle)
            self.card_pub.publish(msg_card)

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
