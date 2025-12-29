import os
import time
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

# Set Mock factory before importing gpiozero anywhere
os.environ['GPIOZERO_PIN_FACTORY'] = 'mock'

from xpi_sensors.sonar_node import SonarNode

class TestSonarNode:
    @classmethod
    def setup_class(cls):
        # Initialize ROS2 context once for the class
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def teardown_class(cls):
        # Shutdown ROS2 context
        if rclpy.ok():
            rclpy.shutdown()

    def test_node_initialization(self):
        """Test if the node can be initialized with mock pins"""
        node = SonarNode()
        assert node.get_name() == 'sonar_node'
        assert node.sensor is not None
        node.destroy_node()

    def test_range_publication(self):
        """Test if the node publishes Range messages"""
        node = SonarNode()
        
        received_msgs = []
        def cb(msg):
            received_msgs.append(msg)

        # Create a temporary subscription to check output
        test_node = Node('test_listener')
        test_node.create_subscription(Range, '/sonar_node/range', cb, 10)

        # Give it some time to spin and publish
        # We manually call timer_callback to avoid waiting for real time
        node.timer_callback()
        
        # Spin once to process subscription
        rclpy.spin_once(test_node, timeout_sec=0.1)

        assert len(received_msgs) > 0
        msg = received_msgs[0]
        assert isinstance(msg, Range)
        assert msg.min_range == 0.02
        assert msg.max_range == 2.0 # Default value
        
        node.destroy_node()
        test_node.destroy_node()
