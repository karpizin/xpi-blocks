# RCWL-0516 Doppler Radar Motion Sensor

The RCWL-0516 is a microwave radar sensor that uses Doppler effect to detect moving objects. It can detect movement through walls, glass, and other non-metallic materials.

## 🔌 Wiring
| RCWL-0516 Pin | RPi GPIO (Physical) | Description |
| :--- | :--- | :--- |
| **VIN** | 5V (Pin 2/4) | Power supply (4-28V) |
| **GND** | GND (Pin 6) | Ground |
| **OUT** | GPIO 17 (Pin 11) | Digital Output (High on motion) |
| **3V3** | - | Output only (Not used) |
| **CDS** | - | LDR input (Optional) |

## 🚀 Quick Start
```bash
ros2 run xpi_sensors rcwl0516_node --ros-args -p gpio_pin:=17
```

## 📡 Interface
### Topics
| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `~/motion` | `std_msgs/Bool` | Output | `True` when motion is detected. |

### Parameters
*   `gpio_pin` (int, default: `17`): GPIO pin connected to OUT.
*   `active_state` (bool, default: `True`): Logical level for motion.
