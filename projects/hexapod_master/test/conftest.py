"""Run the hexapod tests without ROS: put scripts/ and nodes/ on sys.path and install
the rclpy stub from tests/ros_stub (with a real ROS 2 install, the real rclpy is used)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(PROJECT))
for sub in ('scripts', 'nodes'):
    sys.path.insert(0, os.path.join(PROJECT, sub))

try:
    import rclpy  # noqa: F401  (real ROS 2 available)
except ImportError:
    sys.path.insert(0, os.path.join(REPO, 'tests'))
    import ros_stub
    ros_stub.install(os.path.join(REPO, 'src'))
