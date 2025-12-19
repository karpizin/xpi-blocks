# AS7341: 11-Channel Spectral Color Sensor

The **AS7341** is a multi-channel spectrometer. Unlike standard RGB sensors, it measures 8 specific wavelengths of visible light, plus Clear, Near-Infrared (NIR), and Flicker detection.

**Channels:**
1.  F1 (415nm - Violet)
2.  F2 (445nm - Indigo)
3.  F3 (480nm - Blue)
4.  F4 (515nm - Cyan)
5.  F5 (555nm - Green)
6.  F6 (590nm - Yellow)
7.  F7 (630nm - Orange)
8.  F8 (680nm - Red)
9.  Clear
10. NIR (Near-Infrared)

This is ideal for precise color matching, analyzing light sources, or chemical analysis (colorimetry).

## ⚡ Wiring (I2C)

| AS7341 Pin | Raspberry Pi Pin | Description          |
| :---       | :---             | :---                 |
| **VIN**    | 3.3V (Pin 1)     | Power (3.3V)         |
| **GND**    | GND (Pin 6)      | Ground               |
| **SDA**    | GPIO 2 (Pin 3)   | I2C Data             |
| **SCL**    | GPIO 3 (Pin 5)   | I2C Clock            |

> **Note:** Most modules (Adafruit/Sparkfun) have voltage regulators and can take 3.3V or 5V, but 3.3V is safer for logic levels.

## 📦 Bill of Materials
*   1x AS7341 Breakout Board
*   4x Jumper Wires

## 🚀 Usage

### 1. Launch the Node
```bash
ros2 launch xpi_sensors as7341.launch.py
```

### 2. Verify Data
```bash
ros2 topic echo /as7341/spectrum
```
*Output: A generic Float32MultiArray containing raw counts for all channels.*

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `polling_rate` | float | `10.0` | Hz to poll the sensor. |
| `gain` | int | `3` | Sensor gain (0=0.5x, ... 10=512x). |
| `integration_time` | int | `29` | Integration time steps (0-255). |

## 🧩 Topics Interface

### Publishers
*   `~/spectrum` (`std_msgs/msg/Float32MultiArray`)
    *   Layout: `[F1, F2, F3, F4, F5, F6, F7, F8, Clear, NIR]`
    *   Values are raw 16-bit counts.
