# MAX44009: High Dynamic Range Light Sensor

The **MAX44009** is an ultra-low power ambient light sensor with an incredibly wide dynamic range (0.045 Lux to 188,000 Lux). It simulates the human eye's spectral response.

Unlike standard LDRs or cheaper sensors, the MAX44009 uses an internal auto-ranging mechanism, making it perfect for:
*   Outdoor weather stations (Bright sun).
*   Night-time monitoring (Low light).
*   Display backlight control.

## ⚡ Wiring (I2C)

| MAX44009 Pin | Raspberry Pi Pin | Description          |
| :---         | :---             | :---                 |
| **VCC**      | 3.3V (Pin 1)     | Power (1.7V - 3.6V)  |
| **GND**      | GND (Pin 6)      | Ground               |
| **SDA**      | GPIO 2 (Pin 3)   | I2C Data             |
| **SCL**      | GPIO 3 (Pin 5)   | I2C Clock            |

> **Note:** Default I2C address is usually **0x4A**. Some modules allow selecting **0x4B** via an ADDR pin.

## 🚀 Usage

### 1. Launch the Node
```bash
ros2 launch xpi_sensors max44009.launch.py
```

### 2. Verify Data
```bash
ros2 topic echo /max44009/illuminance
```
*Output: Lux value (float).*

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `i2c_bus` | int | `1` | I2C bus number. |
| `i2c_address` | int | `0x4A` | Sensor I2C address (0x4A or 0x4B). |
| `polling_rate` | float | `5.0` | Hz to poll the sensor. |
| `mode` | string | `auto` | 'auto' (automatic range) or 'manual'. |

## 🧩 Topics Interface

### Publishers
*   `~/illuminance` (`sensor_msgs/msg/Illuminance`)
    *   `illuminance`: Ambient light in Lux.
    *   `variance`: 0.0 (Unmeasured).
