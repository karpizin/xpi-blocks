# XPI Tools: CLI Utilities

This package contains command-line tools for monitoring and debugging your XPI robot.

## 🖥️ xpi-top (Sensor Monitor)

**xpi-top** is a TUI (Text User Interface) application similar to `htop`, but for ROS2 sensors.
It automatically scans for active topics and displays their values in a real-time table.

### Features
*   **Auto-Discovery:** Finds topics publishing `sensor_msgs` or `std_msgs`.
*   **Real-time:** Updates at 2Hz.
*   **SSH Friendly:** Works in any terminal via SSH.

### Usage
```bash
ros2 run xpi_tools xpi-top
```

### Supported Types
*   `sensor_msgs/Temperature`
*   `sensor_msgs/Illuminance`
*   `sensor_msgs/BatteryState`
*   `sensor_msgs/Range`
*   `sensor_msgs/FluidPressure`
*   `sensor_msgs/RelativeHumidity`
*   `std_msgs/Float32` / `Int32` / `String`

### Controls
*   `q`: Quit
