import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import threading
import time
import os
import sys

# Add nodes and scripts paths
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'nodes'))

from hexapod_gait_node import HexapodGaitNode

class TestGaitNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_node_publication(self):
        """Test if HexapodGaitNode publishes gait offsets when receiving cmd_vel."""
        node = HexapodGaitNode()
        
        # We need a subscriber to verify output
        received_msgs = []
        def sub_cb(msg):
            received_msgs.append(msg)
            
        test_sub = node.create_subscription(Point, '/hexapod/rf/gait_offset', sub_cb, 10)
        
        # Create a publisher to trigger the node
        vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Spin node in a thread
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        
        # Publish velocity
        msg = Twist()
        msg.linear.x = 0.1
        vel_pub.publish(msg)
        
        # Wait for messages to be processed
        time.sleep(0.5)
        
        # Cleanup
        node.destroy_node()
        
        # Verify
        self.assertTrue(len(received_msgs) > 0, "No gait offset messages received!")

if __name__ == '__main__':
    unittest.main()
