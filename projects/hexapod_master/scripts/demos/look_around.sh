#!/bin/bash
echo "Starting Hexapod Look Around..."

# Turn left (~30 deg)
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: 0.0, y: 0.0, z: 0.258, w: 0.966}}"
sleep 2

# Turn right (~30 deg)
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: 0.0, y: 0.0, z: -0.258, w: 0.966}}"
sleep 2

# Return to center
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
echo "Done."
