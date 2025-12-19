# APDS-9960: RGB, Gesture & Proximity Sensor

This block integrates the **APDS-9960**, a versatile I2C sensor that measures:
1.  **Ambient Light & RGB Color** (Red, Green, Blue, Clear).
2.  **Proximity** (Distance detection up to ~10-20cm).
3.  **Gestures** (Swipe Up, Down, Left, Right, Near, Far).

It is commonly used in smartphones for screen dimming and gesture control, making it perfect for interactive robots.

## ⚡ Wiring (I2C)

| APDS-9960 Pin | Raspberry Pi Pin | Description          |
| :---          | :---             | :---                 |
| **VCC**       | 3.3V (Pin 1)     | Power (3.3V ONLY!)   |
| **GND**       | GND (Pin 6)      | Ground               |
| **SDA**       | GPIO 2 (Pin 3)   | I2C Data             |
| **SCL**       | GPIO 3 (Pin 5)   | I2C Clock            |
| **INT**       | GPIO 4 (Pin 7)   | Interrupt (Optional) |

> **⚠️ WARNING:** Connect VCC to **3.3V**. Connecting to 5V will damage the sensor.

## 📦 Bill of Materials
*   1x APDS-9960 Module (GY-9960)
*   4x Jumper Wires (Female-Female)

## 🚀 Usage

### 1. Launch the Node
```bash
ros2 launch xpi_sensors apds9960.launch.py
```

### 2. Verify Data
**Color Data:**
```bash
ros2 topic echo /apds9960/color_raw
```
*Output (r, g, b, clear values).*

**Proximity:**
```bash
ros2 topic echo /apds9960/proximity
```
*Output (Range 0-255, where 255 is closest).*

**Gestures:**
Move your hand ~5-10cm above the sensor.
```bash
ros2 topic echo /apds9960/gesture
```
*Output: `data: "UP"`, `data: "LEFT"`, etc.*

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `i2c_bus` | int | `1` | I2C bus number (usually 1 on RPi). |
| `polling_rate` | float | `10.0` | Hz to poll color/prox data. |
| `enable_color` | bool | `True` | Enable RGB sensing. |
| `enable_prox` | bool | `True` | Enable Proximity sensing. |
| `enable_gesture` | bool | `True` | Enable Gesture sensing. |

## 🧩 Topics Interface

### Publishers
*   `~/color_raw` (`xpi_interfaces/msg/ColorRGBA` or `std_msgs/ColorRGBA`) - Raw RGBC data.
*   `~/proximity` (`sensor_msgs/msg/Range`) - Proximity distance (simulated in meters or raw units).
*   `~/gesture` (`std_msgs/msg/String`) - Recognized gesture name.
