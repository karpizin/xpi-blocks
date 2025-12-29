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
        
        # Mocking distance reading for gpiozero
        # In mock mode, we can manually set the value that the sensor would 'see'
        if node.sensor:
            # For DistanceSensor in MockFactory, we need to mock the echo pin behavior
            # or simply mock the distance property if possible. 
            # However, simpler is to just mock the publish call if we want to test ROS logic,
            # or correctly setup Mock pins.
            pass

        received_msgs = []
        def cb(msg):
            received_msgs.append(msg)

        # Create a temporary subscription to check output
        test_node = Node('test_listener')
        # Note: the topic is relative or absolute. In node it's '~/range' -> '/sonar_node/range'
        test_node.create_subscription(Range, '/sonar_node/range', cb, 10)

        # To avoid blocking on self.sensor.distance, we can mock the sensor object itself 
        # or ensure the mock pins respond. 
        # Let's use a simpler approach: mock the distance attribute
        if node.sensor:
            type(node.sensor).distance = property(lambda x: 0.5) # Simulate 0.5m

        # Manually call timer_callback
        node.timer_callback()
        
        # Spin to process
        rclpy.spin_once(test_node, timeout_sec=0.5)

        assert len(received_msgs) > 0
        msg = received_msgs[0]
        assert msg.range == 0.5
        
        node.destroy_node()
        test_node.destroy_node()
