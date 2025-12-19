# TCS34725 RGB Color Sensor

The TCS34725 is a widely used RGB color light to digital converter with IR filter. It provides Red, Green, Blue, and Clear light sensing values. The IR blocking filter allows the sensor to perform color sensing that accurately matches the human eye.

![TCS34725](https://cdn-shop.adafruit.com/970x728/1334-00.jpg)
*(Image is illustrative, generic module)*

## ⚡ Wiring

The TCS34725 communicates via I2C.

| TCS34725 Pin | Raspberry Pi Pin | Description |
| :--- | :--- | :--- |
| **VIN** | 3.3V (Pin 1) | Power |
| **GND** | GND (Pin 6) | Ground |
| **SDA** | GPIO 2 (Pin 3) | I2C Data |
| **SCL** | GPIO 3 (Pin 5) | I2C Clock |
| **LED** | (Optional) | Controls built-in white LED (if equipped) |

**I2C Address:** Usually **0x29** (Fixed).

## 📦 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `i2c_bus` | int | `1` | I2C bus number |
| `i2c_address` | int | `0x29` | I2C address |
| `poll_rate` | float | `2.0` | Read frequency in Hz |
| `gain` | int | `1` | Analog Gain: `1`, `4`, `16`, `60` (High for low light) |
| `integration_time_ms` | int | `154` | Integration time: `2.4`, `24`, `50`, `101`, `154`, `700` |

## 🚀 Usage

**Run the node:**
```bash
ros2 launch xpi_sensors tcs34725.launch.py
```

**Verify output:**
```bash
# Check color (Normalized RGBA)
ros2 topic echo /tcs34725/color

# Check Lux
ros2 topic echo /tcs34725/illuminance

# Check Color Temperature (Kelvin)
ros2 topic echo /tcs34725/color_temp
```

## 📝 Notes
*   **Normalized Color:** The `/color` topic outputs `r`, `g`, `b` values normalized by the `Clear` channel intensity. This represents the "chromaticity" or color ratios, useful for identifying object colors regardless of brightness.
*   **Calibration:** For precise color matching (e.g., "Is this red or orange?"), you typically need to calibrate the raw values against known samples in your specific lighting conditions.
