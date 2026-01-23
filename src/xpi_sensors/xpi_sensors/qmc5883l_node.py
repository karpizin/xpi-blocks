#!/usr/bin/env python3
import math
import time
import smbus2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float32

class QMC5883LNode(Node):
# ... (intermediate code remains the same)
            # 2. Calculate Heading
            heading = math.atan2(y, x) + self.declination
            
            # Correct for wrap around
            if heading < 0:
                heading += 2 * math.pi
            if heading > 2 * math.pi:
                heading -= 2 * math.pi
            
            # Convert to degrees
# ... (intermediate code remains the same)
def main(args=None):
    rclpy.init(args=args)
    node = QMC5883LNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
