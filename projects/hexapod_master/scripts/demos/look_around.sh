#!/bin/bash
echo "Starting Hexapod Look Around..."

# Поворот влево (~30 град)
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: 0.0, y: 0.0, z: 0.258, w: 0.966}}"
sleep 2

# Поворот вправо (~30 град)
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: 0.0, y: 0.0, z: -0.258, w: 0.966}}"
sleep 2

# Возврат в центр
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
echo "Done."
