# OPT3001 Ambient Light Sensor

The OPT3001 is a sensor that measures the intensity of visible light. The spectral response of the sensor tightly matches the photopic response of the human eye and includes significant infrared rejection.

![OPT3001](https://cdn-shop.adafruit.com/970x728/1980-00.jpg) 
*(Image is illustrative, generic OPT3001 module)*

## ⚡ Wiring

The OPT3001 communicates via I2C.

| OPT3001 Pin | Raspberry Pi Pin | Description |
| :--- | :--- | :--- |
| **VCC** | 3.3V (Pin 1 or 17) | Power (1.6V - 3.6V) |
| **GND** | GND (Pin 6, 9, etc) | Ground |
| **SDA** | GPIO 2 (Pin 3) | I2C Data |
| **SCL** | GPIO 3 (Pin 5) | I2C Clock |
| **ADDR** | GND | Sets address to 0x44 (See below) |

**Address Selection (ADDR Pin):**
*   **GND:** 0x44 (Default)
*   **VCC:** 0x45
*   **SDA:** 0x46
*   **SCL:** 0x47

## 📦 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `i2c_bus` | int | `1` | I2C bus number (usually 1 on RPi) |
| `i2c_address` | int | `0x44` | I2C address of the sensor |
| `poll_rate` | float | `1.0` | Read frequency in Hz |
| `frame_id` | string | `opt3001_link` | TF frame ID for the header |

## 🚀 Usage

**Run the node:**
```bash
ros2 launch xpi_sensors opt3001.launch.py
```

**Verify output:**
```bash
ros2 topic echo /opt3001/illuminance
```

Output should look like:
```yaml
header:
  stamp:
    sec: 1629...
    nanosec: ...
  frame_id: opt3001_link
illuminance: 120.5
variance: 0.0
```

## 📝 Notes
*   This driver uses the **Automatic Full-Scale Range** setting of the OPT3001.
*   Integration time is set to **800ms** by default for high stability and low-light performance.
*   Lux calculation formula: `lux = 0.01 * (2 ^ exponent) * mantissa`.
