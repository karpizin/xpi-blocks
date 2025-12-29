#!/bin/bash
# Sequentially lifts each leg to test range of motion

echo "--- Hexapod Individual Leg Calibration ---"

for i in {0..5}
do
   echo "Testing Leg $i..."
   ros2 topic pub --once /hexapod/terrain_feedback geometry_msgs/msg/Point "{x: $i, z: 0.05}"
   sleep 1
   ros2 topic pub --once /hexapod/terrain_feedback geometry_msgs/msg/Point "{x: $i, z: 0.0}"
   sleep 0.5
done

echo "Calibration sequence complete."
