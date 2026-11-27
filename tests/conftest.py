"""Shared fixtures for tests that run ROS 2 nodes without ROS (see ros_stub/)."""
import os
import sys
import time

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, 'src')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
for pkg in sorted(os.listdir(SRC)):
    if os.path.isfile(os.path.join(SRC, pkg, 'setup.py')):
        sys.path.insert(0, os.path.join(SRC, pkg))

# projects/<robot>/nodes|scripts are plain scripts (run with python3 by their launch files)
for proj in sorted(os.listdir(os.path.join(REPO, 'projects'))):
    for sub in ('scripts', 'nodes'):
        d = os.path.join(REPO, 'projects', proj, sub)
        if os.path.isdir(d):
            sys.path.insert(0, d)

import ros_stub  # noqa: E402
from ros_stub import rclpy_stub  # noqa: E402

# get_package_share_directory('<pkg>') -> src/<pkg> (web_static/, config/ live there)
ros_stub.install(SRC)

# Never touch real I2C buses from a test run, even on a Raspberry Pi
os.environ.setdefault('XPI_MOCK_I2C', '1')

_real_sleep = time.sleep


@pytest.fixture
def fast_sleep(monkeypatch):
    """Cap time.sleep so sensor warm-up delays do not slow the suite down."""
    monkeypatch.setattr(time, 'sleep', lambda s=0: _real_sleep(min(max(s, 0), 0.01)))


@pytest.fixture
def ros(fast_sleep):
    """Clean rclpy stub state per test; yields the stub module."""
    rclpy_stub.OVERRIDES.clear()
    rclpy_stub.NODES.clear()
    rclpy_stub.LOG.clear()
    yield rclpy_stub
    for n in list(rclpy_stub.NODES):
        if not n.destroyed:
            try:
                n.destroy_node()
            except Exception:  # noqa: BLE001 - best-effort cleanup
                pass
