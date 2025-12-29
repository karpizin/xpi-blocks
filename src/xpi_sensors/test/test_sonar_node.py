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
        from unittest.mock import MagicMock
        node = SonarNode()
        
        # Replace the real sensor with a mock
        mock_sensor = MagicMock()
        mock_sensor.distance = 0.5
        node.sensor = mock_sensor

        received_msgs = []
        def cb(msg):
            received_msgs.append(msg)

        test_node = Node('test_listener')
        test_node.create_subscription(Range, '/sonar_node/range', cb, 10)

        # Manually call timer_callback
        node.timer_callback()
        
        # Spin to process
        rclpy.spin_once(test_node, timeout_sec=0.5)

        assert len(received_msgs) > 0
        msg = received_msgs[0]
        assert msg.range == 0.5
        
        node.destroy_node()
        test_node.destroy_node()
