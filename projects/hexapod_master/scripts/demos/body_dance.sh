#!/bin/bash
echo "Hexapod Body Dance initiated..."

# Наклон вперед-влево
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: 0.1, y: 0.1, z: 0.0, w: 0.99}}"
sleep 1

# Наклон вперед-вправо
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: -0.1, y: 0.1, z: 0.0, w: 0.99}}"
sleep 1

# Наклон назад-вправо
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: -0.1, y: -0.1, z: 0.0, w: 0.99}}"
sleep 1

# Наклон назад-влево
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: 0.1, y: -0.1, z: 0.0, w: 0.99}}"
sleep 1

# Возврат
ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
echo "Dance finished."
