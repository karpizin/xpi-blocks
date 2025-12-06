# HC-SR04 Ultrasonic Range Sensor

This block provides a driver for the popular, low-cost HC-SR04 ultrasonic distance sensor.

## 📦 Bill of Materials
*   Raspberry Pi
*   HC-SR04 Module
*   **Voltage Divider** (2 resistors: 1kΩ & 2kΩ) OR Level Shifter
    *   *Warning:* The Echo pin outputs 5V, but Raspberry Pi GPIO handles max 3.3V. **You will fry your Pi without a divider!**

## 🔌 Wiring

| HC-SR04 | Raspberry Pi | Component |
|Data | GPIO 23 (Pin 16) | Direct connection |
| Echo | GPIO 24 (Pin 18) | **Via Voltage Divider!** |
| VCC | 5V (Pin 2) | |
| GND | GND (Pin 6) | Common Ground |

### Voltage Divider Logic
1.  HC-SR04 **Echo** -> 1kΩ Resistor -> RPi **GPIO 24**.
2.  RPi **GPIO 24** -> 2kΩ Resistor -> **GND**.

## 🚀 Quick Start
```bash
ros2 launch xpi_sensors sonar.launch.py trigger_pin:=23 echo_pin:=24
```

## 📡 Interface
### Publishers
*   `~/range` (`sensor_msgs/Range`): Distance in meters.

### Parameters
*   `trigger_pin` (int, default: 23)
*   `echo_pin` (int, default: 24)
*   `max_distance` (float, default: 2.0): Max reliable range in meters.
*   `update_rate` (float, default: 10.0): Hertz.

## ⚠️ Troubleshooting
*   **Sensor reads 0/Max constantly?** Check your wiring. Ensure the sensor has 5V power (not 3.3V), but Echo is level-shifted to 3.3V.
*   **High Latency?** Ultrasonic sensors are slow by physics (sound speed). Do not set `update_rate` > 20Hz for long distances.
