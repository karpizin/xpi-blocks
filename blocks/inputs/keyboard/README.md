# Keyboard as Gamepad (Input)

This block turns your computer keyboard into a virtual gamepad/joystick for your robot. Unlike standard teleop packages, this node publishes `sensor_msgs/Joy`, allowing you to use the same `joy_mapper` configuration for both physical gamepads and the keyboard.

## 🎯 Features
*   **WASD Navigation:** Maps `W/A/S/D` to Analog Axes (Left Stick & Right Stick).
*   **Button Mapping:** Maps `Space`, `Enter`, `1-4` to Joystick Buttons.
*   **Unified Logic:** Works seamlessly with `joy_mapper_node`.

## 📦 Requirements
*   `xterm` (for capturing key presses in a separate window): `sudo apt install xterm`

## 🚀 Usage

### 1. Launch
```bash
ros2 launch xpi_inputs keyboard.launch.py
```
This opens a small `xterm` window. **Click on it** to focus.

### 2. Controls
| Key | Joystick Equivalent | Value |
| :--- | :--- | :--- |
| **W / S** | Axis 1 (Left Stick Y) | +1.0 / -1.0 |
| **A / D** | Axis 3 (Right Stick X) | +1.0 / -1.0 |
| **Space** | Button 0 (X / A) | 1 (Held) |
| **Enter** | Button 1 (O / B) | 1 (Held) |
| **E** | Button 2 (Triangle) | 1 (Held) |
| **Q** | Button 3 (Square) | 1 (Held) |
| **1, 2, 3, 4** | Buttons 4-7 | 1 (Held) |

### 3. Drive Robot (Integration)
Since this node mimics a gamepad (Axis 1 = Throttle, Axis 3 = Steering), you can use it with the standard PS4/Web mapper config:

```bash
# Terminal 1: Keyboard Input
ros2 launch xpi_inputs keyboard.launch.py

# Terminal 2: Mapper (converts to cmd_vel)
ros2 run xpi_inputs joy_mapper_node --ros-args --params-file src/xpi_inputs/config/mapper_web.yaml
```

## 📡 Topic
*   `/joy` (`sensor_msgs/Joy`)

## ⚠️ Notes
*   Keys must be held down to maintain axis value (like pushing a stick).
*   Releasing a key immediately centers the axis (0.0).