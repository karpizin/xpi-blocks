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
Use the standard `/cmd_vel` topic. The robot now supports **Omnidirectional movement** (walking sideways and backwards simultaneously) and **Rotation**.
```bash
# Move forward at 0.1 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# Walk sideways (Right)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {y: -0.05}}"

# Rotate while moving
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.2}}"
```

### Advanced Gait Selection
Change the gait type dynamically via ROS2 parameters:
```bash
# Available: tripod, wave, ripple, amble
ros2 param set /hexapod_gait_node gait_type "amble"
```

### Turning to a Specific Angle (Heading Control)
The robot can autonomously turn to a specific compass heading (Yaw) using IMU feedback.
```bash
# Turn to 90 degrees (Left) - Target is in RADIANS
ros2 topic pub --once /hexapod/target_heading std_msgs/msg/Float32 "{data: 1.57}"

# Return to 0 (Facing forward)
ros2 topic pub --once /hexapod/target_heading std_msgs/msg/Float32 "{data: 0.0}"
```

### Circular Movement
The robot can follow a circular path by specifying a radius and speed.
```bash
# 1. Set the radius (e.g., 1 meter). Positive = Left turn, Negative = Right.
ros2 topic pub --once /hexapod/set_circle_radius std_msgs/msg/Float32 "{data: 1.0}"

# 2. Set the speed to start moving (e.g., 0.1 m/s)
ros2 topic pub --once /hexapod/set_circle_speed std_msgs/msg/Float32 "{data: 0.1}"

# 3. To stop, set speed to 0
ros2 topic pub --once /hexapod/set_circle_speed std_msgs/msg/Float32 "{data: 0.0}"
```

### Terrain Adaptation
To manually test uneven terrain correction for a specific leg:
```bash
# Format: x=leg_index (0-5), z=offset in meters
# Lift Right Front (0) leg by 2cm to adapt to a bump
ros2 topic pub --once /hexapod/terrain_feedback geometry_msgs/msg/Point "{x: 0, z: 0.02}"
```

### Body Pose Control
Use the `/hexapod/body_pose` topic:
```bash
# Squat (lower body by 5cm)
ros2 topic pub --once /hexapod/body_pose geometry_msgs/msg/Pose "{position: {z: -0.05}}"

# Tilt forward (Pitch)
ros2 topic pub --once /hexapod/body_pose geometry_msgs/msg/Pose "{orientation: {x: 0.0, y: 0.1, z: 0.0, w: 0.99}}"
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