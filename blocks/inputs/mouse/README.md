# Mouse & Touchpad Input

This block allows you to use a standard USB/Bluetooth mouse or touchpad as a high-precision input device for your robot. Unlike a joystick, a mouse provides relative motion data, which can be interpreted in different ways.

## 🎯 Features
*   **Velocity Mode:** Moving the mouse drives the robot. Stopping the mouse stops the robot (auto-centering logic).
*   **Position Mode:** Accumulates movement to control absolute position (e.g., Pan/Tilt camera, Robot Arm).
*   **Headless:** Uses `evdev` to read directly from `/dev/input/`, working without a GUI/Desktop environment.

## 📦 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `device_name` | string | `Mouse` | Substring to match device name (e.g., "Logitech", "Touchpad") |
| `device_path` | string | `""` | Specific path (e.g., `/dev/input/event3`). Overrides name. |
| `mode` | string | `velocity` | `velocity` (auto-zeroing) or `position` (accumulating) |
| `sensitivity` | float | `0.005` | Input scaling factor |
| `decay` | float | `0.2` | Decay rate for velocity mode (0.0-1.0). Higher = faster stop. |

## 🎮 Mapping (Standard)

The node publishes a standard `sensor_msgs/Joy` message.

*   **Axis 0:** X Movement (Left/Right)
*   **Axis 1:** Y Movement (Up/Down)
*   **Axis 2:** Wheel Vertical
*   **Button 0:** Left Click
*   **Button 1:** Right Click
*   **Button 2:** Middle Click

## 🚀 Usage

**1. Find your device name (optional):**
```bash
cat /proc/bus/input/devices
```

**2. Launch the node:**
```bash
ros2 launch xpi_inputs mouse.launch.py
```

**3. Use with Joy Mapper:**
To control a robot, combine this with `joy_mapper_node`. See `src/xpi_inputs/config/mapper_mouse_pantilt.yaml` for a Pan/Tilt example.

## ⚠️ Permissions
You need permission to read input devices.
```bash
sudo usermod -aG input $USER
```
Then logout and login again.
