#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Point
import math

class ContactBridgeNode(Node):
    """
    Bridges Gazebo contact sensors to the hexapod terrain adaptation logic.
    Converts 'Touch' events into Z-offsets for the Gait Engine.
    """
    def __init__(self):
        super().__init__('contact_bridge_node')

        self.leg_names = ['rf', 'rm', 'rb', 'lf', 'lm', 'lb']
        self.subs = []
        self.feedback_pub = self.create_publisher(Point, '/hexapod/terrain_feedback', 10)

        # Create subscriptions for each leg's contact sensor
        for i, name in enumerate(self.leg_names):
            topic = f'/{name}_contact'
            sub = self.create_subscription(
                ContactsState, 
                topic, 
                lambda msg, idx=i: self.contact_callback(msg, idx), 
                10
            )
            self.subs.append(sub)

        self.get_logger().info('Contact Bridge Node initialized. Monitoring 6 leg sensors.')

    def contact_callback(self, msg, leg_index):
        """
        If contact is detected, we assume the leg has hit the ground.
        In a real scenario, we'd use this to stop the downward 'stance' phase early.
        For this simulation demo, we just notify that the leg is 'at zero' level.
        """
        if len(msg.states) > 0:
            # Contact detected!
            # We send a feedback point: x = leg_index, z = 0.0 (meaning 'ground reached')
            # The Gait Node can use this to adapt.
            feedback = Point()
            feedback.x = float(leg_index)
            feedback.z = 0.0 # Standard level
            self.feedback_pub.publish(feedback)

def main(args=None):
    rclpy.init(args=args)
    node = ContactBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
