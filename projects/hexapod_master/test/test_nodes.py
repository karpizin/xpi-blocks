import time
import unittest

import rclpy
from geometry_msgs.msg import Point, Twist
from hexapod_gait_node import HexapodGaitNode


class TestGaitNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_node_publication(self):
        """HexapodGaitNode publishes leg offsets after receiving cmd_vel."""
        node = HexapodGaitNode()
        received = []
        node.create_subscription(Point, '/hexapod/rf/gait_offset', received.append, 10)
        vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

        msg = Twist()
        msg.linear.x = 0.1
        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            vel_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)

        node.destroy_node()
        self.assertTrue(received, 'No gait offset messages received!')


if __name__ == '__main__':
    unittest.main()
