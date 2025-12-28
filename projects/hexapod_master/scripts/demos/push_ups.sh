#!/bin/bash
echo "Starting Hexapod Push-ups..."

while true; do
    echo "Down..."
    ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{position: {z: -0.04}}"
    sleep 1
    echo "Up..."
    ros2 topic pub --once /hexapod/body_pose geometry_msgs/Pose "{position: {z: -0.08}}"
    sleep 1
done
