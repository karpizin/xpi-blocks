# HDC1080 Temperature & Humidity Sensor

The HDC1080 is a digital humidity sensor with integrated temperature sensor from Texas Instruments. It provides high measurement accuracy at very low power.

## 🔌 Wiring
| HDC1080 Pin | RPi Pin (Physical) | Description |
| :--- | :--- | :--- |
| **VCC** | 3.3V (Pin 1) | Power Supply (2.7V - 5.5V) |
| **GND** | GND (Pin 9) | Ground |
| **SDA** | I2C SDA (Pin 3) | I2C Data |
| **SCL** | I2C SCL (Pin 5) | I2C Clock |

**Default I2C Address:** `0x40`

## 🚀 Quick Start
```bash
ros2 run xpi_sensors hdc1080_node --ros-args -p i2c_address:=0x40
```

## 📡 Interface
### Topics
| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `~/temperature` | `sensor_msgs/Temperature` | Output | Temperature in Celsius. |
| `~/humidity` | `sensor_msgs/RelativeHumidity` | Output | Relative Humidity in %. |

### Parameters
*   `i2c_bus` (int, default: `1`): I2C bus number.
*   `i2c_address` (int, default: `0x40`): Device address.
*   `update_rate` (float, default: `1.0`): Polling frequency in Hz.
