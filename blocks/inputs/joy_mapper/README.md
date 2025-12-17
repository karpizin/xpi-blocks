# Joy Mapper Node

The `joy_mapper_node` is a universal translator that converts raw joystick inputs (`sensor_msgs/Joy`) into meaningful robot commands. It eliminates the need to write custom Python scripts for every new robot or controller configuration.

## 🎯 Features

*   **Arcade Drive Mixing:** Automatically converts two axes (e.g., Left Stick Y + Right Stick X) into `geometry_msgs/Twist` (`cmd_vel`) for differential drive robots.
*   **Flexible Button Bindings:**
    *   **Toggle:** Press once to turn ON, press again to turn OFF (e.g., Headlights).
    *   **Momentary:** ON while held, OFF when released (e.g., Horn).
*   **Axis Mapping:** Map analog triggers or sticks to `Float32` topics (e.g., Servo position, Gripper force).
*   **No-Code Configuration:** All behavior is defined in a simple `yaml` file.

## 📦 Parameters & Configuration

Configuration is done via a YAML file passed to the node.

### 1. Drive Control (Arcade/Tank)

```yaml
enable_drive: true        # Enable cmd_vel publishing
drive_type: "arcade"      # Currently only 'arcade' is supported
axis_linear: 1            # Axis index for Forward/Back speed
axis_angular: 3           # Axis index for Turning
scale_linear: 1.0         # Max linear speed (m/s)
scale_angular: 1.5        # Max angular speed (rad/s)
deadzone: 0.05            # Ignore small stick movements
```

### 2. Bindings

The `bindings` parameter is a list of strings using the format:
`"TYPE:INDEX:TOPIC:MSG_TYPE:PARAM"`

| Type | Description | Message Type | Param | Example |
| :--- | :--- | :--- | :--- | :--- |
| `toggle_btn` | Toggles output (True/False) on button press | `Bool` | (Ignored) | `"toggle_btn:0:lights:Bool"` |
| `momentary_btn` | True when held, False when released | `Bool` | (Ignored) | `"momentary_btn:1:horn:Bool"` |
| `axis` | Maps axis value directly to topic | `Float32` | Multiplier | `"axis:5:gripper:Float32:180.0"` |

*   **INDEX:** The index of the button or axis in the `joy.buttons[]` or `joy.axes[]` array. Use `ros2 topic echo /joy` to find these indices for your controller.
*   **TOPIC:** The output ROS2 topic name (relative to node namespace).
*   **MSG_TYPE:** Currently supports `Bool`, `Float32`, `Int32`.
*   **PARAM:**
    *   For `axis`, this is a multiplier. If axis is 0..1 and param is 180, output is 0..180. Use negative param to invert direction.

## 🚀 Usage

### 1. Identify your controller Mapping
Run the joystick driver and press buttons to see their indices:
```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

### 2. Create/Edit Config
Edit `src/xpi_inputs/config/mapper_example.yaml` or create your own.

### 3. Run the Mapper
```bash
ros2 launch xpi_inputs mapper.launch.py
```

## 💡 Example Scenarios

### Simple Robot (2WD + Headlights)
*   **Left Stick Y:** Throttle
*   **Right Stick X:** Steering
*   **Button A:** Headlights (Toggle)

**Config:**
```yaml
joy_mapper_node:
  ros__parameters:
    enable_drive: true
    axis_linear: 1
    axis_angular: 3
    bindings:
      - "toggle_btn:0:headlights:Bool"
```

### Robot Arm (Servo Control)
*   **Right Trigger (Axis 5):** Claw Open/Close (0-180 degrees)
*   **Left Stick X (Axis 0):** Base Rotation (-90 to +90 degrees)

**Config:**
```yaml
joy_mapper_node:
  ros__parameters:
    enable_drive: false
    bindings:
      - "axis:5:claw_pos:Float32:180.0"
      - "axis:0:base_rot:Float32:-90.0"
```
