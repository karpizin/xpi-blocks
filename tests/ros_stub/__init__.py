"""Minimal ROS 2 Humble stand-in for smoke tests: see messages.py and rclpy_stub.py."""
import os
import sys
import types

from . import messages, rclpy_stub


def _ament_index(share_root):
    mod = types.ModuleType('ament_index_python')
    pk = types.ModuleType('ament_index_python.packages')

    class PackageNotFoundError(KeyError):
        pass

    def get_package_share_directory(name):
        path = os.path.join(share_root, name)
        if not os.path.isdir(path):
            raise PackageNotFoundError(name)
        return path

    pk.get_package_share_directory = get_package_share_directory
    pk.PackageNotFoundError = PackageNotFoundError
    mod.packages = pk
    sys.modules['ament_index_python'] = mod
    sys.modules['ament_index_python.packages'] = pk


def _cv_bridge():
    mod = types.ModuleType('cv_bridge')

    class CvBridgeError(TypeError):
        pass

    class CvBridge:
        def imgmsg_to_cv2(self, msg, desired_encoding='passthrough'):
            import numpy
            return numpy.zeros((max(msg.height, 1), max(msg.width, 1), 3), dtype=numpy.uint8)

        def cv2_to_imgmsg(self, img, encoding='passthrough', header=None):
            from sensor_msgs.msg import Image
            m = Image()
            m.height, m.width = int(img.shape[0]), int(img.shape[1])
            m.encoding = 'bgr8' if encoding == 'passthrough' else encoding
            return m

        def compressed_imgmsg_to_cv2(self, msg, desired_encoding='passthrough'):
            import numpy
            return numpy.zeros((1, 1, 3), dtype=numpy.uint8)

    mod.CvBridge, mod.CvBridgeError = CvBridge, CvBridgeError
    sys.modules['cv_bridge'] = mod


def install(share_root):
    """Install stub modules. ``share_root/<pkg>`` plays the role of share/<pkg>."""
    messages.install()
    rclpy_stub.install()
    _ament_index(share_root)
    _cv_bridge()
