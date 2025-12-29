#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Pose
from std_msgs.msg import Float32, String
import time
import math

class TestScenarioNode(Node):
    """
    Automated Test Suite for Hexapod Master.
    Executes a sequence of movement and manipulation tests.
    """
    def __init__(self):
        super().__init__('test_scenario_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.heading_pub = self.create_publisher(Float32, '/hexapod/target_heading', 10)
        self.pose_pub = self.create_publisher(Pose, '/hexapod/body_pose', 10)
        self.terrain_pub = self.create_publisher(Point, '/hexapod/terrain_feedback', 10)
        
        # Test sequences
        self.tests = [
            {'name': 'Body Height Test', 'func': self.test_body_height},
            {'name': 'Leg Range Test (All Legs)', 'func': self.test_leg_ranges},
            {'name': 'Walking: Forward 2 seconds', 'func': self.test_walk_forward},
            {'name': 'Walking: Sideways 2 seconds', 'func': self.test_walk_sideways},
            {'name': 'Rotation: Turn 90 degrees', 'func': self.test_rotate},
            {'name': 'Complex: Circular Path', 'func': self.test_circular},
            {'name': 'Reset to Idle', 'func': self.reset_robot}
        ]

        self.get_logger().info('Hexapod Test Scenario Node ready.')
        self.get_logger().info('Publish to /hexapod/run_test "all" or "test_name" to start.')
        
        self.create_subscription(String, '/hexapod/run_test', self.run_test_callback, 10)

    def run_test_callback(self, msg):
        target = msg.data.lower()
        for test in self.tests:
            if target == 'all' or target == test['name'].lower():
                self.get_logger().info(f'--- STARTING TEST: {test["name"]} ---')
                test['func']()
                time.sleep(1.0)
        self.get_logger().info('--- ALL REQUESTED TESTS COMPLETE ---')

    def test_body_height(self):
        # Move body up and down
        for z in [-0.04, -0.10, -0.08]:
            p = Pose()
            p.position.z = z
            self.pose_pub.publish(p)
            time.sleep(1.5)

    def test_leg_ranges(self):
        # Sequentially lift each leg
        for i in range(6):
            # Lift leg
            self.terrain_pub.publish(Point(x=float(i), z=0.04))
            time.sleep(0.5)
            # Drop leg
            self.terrain_pub.publish(Point(x=float(i), z=0.0))
            time.sleep(0.3)

    def test_walk_forward(self):
        msg = Twist()
        msg.linear.x = 0.1
        self.cmd_pub.publish(msg)
        time.sleep(2.0)
        self.stop_robot()

    def test_walk_sideways(self):
        msg = Twist()
        msg.linear.y = 0.05
        self.cmd_pub.publish(msg)
        time.sleep(2.0)
        self.stop_robot()

    def test_rotate(self):
        # Use the Heading Controller
        self.heading_pub.publish(Float32(data=1.57)) # 90 deg
        time.sleep(4.0) # Wait for it to rotate
        self.heading_pub.publish(Float32(data=0.0))  # Back to 0
        time.sleep(2.0)

    def test_circular(self):
        # Move in a small circle for 3 seconds
        radius_pub = self.create_publisher(Float32, '/hexapod/set_circle_radius', 10)
        speed_pub = self.create_publisher(Float32, '/hexapod/set_circle_speed', 10)
        time.sleep(0.5)
        
        radius_pub.publish(Float32(data=0.5))
        speed_pub.publish(Float32(data=0.1))
        time.sleep(5.0)
        
        speed_pub.publish(Float32(data=0.0)) # Stop

    def reset_robot(self):
        self.stop_robot()
        p = Pose()
        p.position.z = -0.08
        self.pose_pub.publish(p)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = TestScenarioNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
