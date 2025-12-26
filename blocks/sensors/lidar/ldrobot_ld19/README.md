# LDROBOT LD19 / D300 2D LiDAR

The LD19 is a compact, high-performance 2D laser range scanner (LiDAR). It performs a 360-degree scan of the environment and outputs point cloud data over UART.

## 🔌 Wiring
| LD19 Pin | RPi Pin (Physical) | Description |
| :--- | :--- | :--- |
| **VCC** | 5V (Pin 2/4) | Power Supply |
| **GND** | GND (Pin 6) | Ground |
| **TX** | GPIO 15 (Pin 10 - RXD) | Data Transmission (from LiDAR to Pi) |
| **PWM** | GPIO 18 (Pin 12) | Speed Control (Optional) |

*Note: Ensure UART is enabled on your Raspberry Pi and the console is disabled.*

## 🚀 Quick Start
```bash
ros2 run xpi_sensors ld19_node --ros-args -p port:=/dev/ttyUSB0
```

## 📡 Interface
### Topics
| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `~/scan` | `sensor_msgs/LaserScan` | Output | 360° laser scan data. |

### Parameters
*   `port` (string, default: `/dev/ttyUSB0`): Serial port path.
*   `frame_id` (string, default: `lidar_link`): TF frame ID.
*   `speed_control` (bool, default: `False`): Enable PWM speed control.
