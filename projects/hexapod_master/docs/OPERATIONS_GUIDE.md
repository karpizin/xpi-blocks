# Hexapod Master Operations Guide

This document describes the architecture of the hexapod control system and instructions for running it.

## 🏗 Control Architecture (Node Hierarchy)

The system is built on a top-down principle:
1.  **Gait Node** (`/cmd_vel`) -> Generates dynamic leg offsets (walking).
2.  **Auto-Leveler Node** (`/imu/data`) -> Generates body correction angles (leveling).
3.  **Body Node** (Hub) -> Aggregates data from Gait and Leveler, calculates final (X,Y,Z) coordinates for each of the 6 legs.
4.  **IK Bridge** -> Translates (X,Y,Z) for each leg into 3 servo angles (Coxa, Femur, Tibia).
5.  **Hardware Driver** -> Sends angles to physical servos or the simulator.

## 🚀 Running the System

### 1. Full Launch (Recommended)
Starts all math and logic nodes with a single command:
```bash
ros2 launch xpi_projects hexapod_full.launch.py
```

### 2. Visualization (Rviz2)
```bash
ros2 launch xpi_projects view_hexapod.launch.py
```

### 3. Simulation (Gazebo)
```bash
ros2 launch xpi_projects gazebo.launch.py
```

## 🎮 Control (Topics)

### Walking (Velocity Control)
Use the standard `/cmd_vel` topic:
```bash
# Move forward at 0.1 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

### Body Pose Control
Use the `/hexapod/body_pose` topic:
```bash
# Squat (lower body by 5cm)
ros2 topic pub /hexapod/body_pose geometry_msgs/msg/Pose "{position: {z: -0.05}}"
```

## 🎭 Demos & Tricks
Ready-to-use scripts for rapid capability demonstration located in `scripts/demos/`:
1.  **Push-ups**: `bash scripts/demos/push_ups.sh`
2.  **Look around**: `bash scripts/demos/look_around.sh`
3.  **Body dance**: `bash scripts/demos/body_dance.sh`

## 🛠 Debugging
*   **Leg not moving:** Check if `ik_bridge` is running. It is the final link before the hardware.
*   **Jerky movements:** Adjust the `speed` parameter in `hexapod_body_kinematics_node.py` (interpolator).
*   **Robot "lagging":** Ensure publishing frequency in all topics is at least 30-50 Hz.