#!/bin/bash
echo "Hexapod Body Dance initiated..."

# Tilt forward-left
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: 0.1, y: 0.1, z: 0.0, w: 0.99}}"
sleep 1

# Tilt forward-right
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: -0.1, y: 0.1, z: 0.0, w: 0.99}}"
sleep 1

# Tilt backward-right
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: -0.1, y: -0.1, z: 0.0, w: 0.99}}"
sleep 1

# Tilt backward-left
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: 0.1, y: -0.1, z: 0.0, w: 0.99}}"
sleep 1

# Return
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
echo "Dance finished."
