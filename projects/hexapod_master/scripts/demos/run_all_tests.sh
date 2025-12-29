#!/bin/bash
# Triggers the full automated test suite

echo "--- Hexapod Full Test Suite Trigger ---"
echo "Make sure test_suite.launch.py is running!"

ros2 topic pub --once /hexapod/run_test std_msgs/msg/String "{data: 'all'}"
