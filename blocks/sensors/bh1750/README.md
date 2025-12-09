# BH1750 Light Sensor

The **BH1750** is a digital Ambient Light Sensor (ALS) with an I2C interface. It measures illuminance in Lux (lx) and is widely used for adjusting screen brightness or detecting environmental lighting conditions.

## ⚡ Specifications
*   **Interface:** I2C (Address `0x23` or `0x5C`)
*   **Voltage:** 3.3V - 5V (Module usually has regulator)
*   **Range:** 1 - 65535 lx
*   **Resolution:** 1 lx (High Res Mode), 0.5 lx (High Res Mode 2), 4 lx (Low Res Mode)

## 🔌 Wiring (Raspberry Pi)

| BH1750 Pin | Raspberry Pi Pin | Note |
| :--- | :--- | :--- |
| **VCC** | 3.3V (Pin 1) | |
| **GND** | GND (Pin 6) | |
| **SCL** | GPIO 3 (SCL) | I2C Bus 1 |
| **SDA** | GPIO 2 (SDA) | I2C Bus 1 |
| **ADDR** | GND / VCC | GND=`0x23`, VCC=`0x5C` |

## 🚀 Usage

### 1. Launch the Node
```bash
ros2 launch xpi_sensors bh1750.launch.py
```
*(Or use the provided `bh1750.launch.py` in this directory if you haven't installed the package globally yet)*

### 2. Verify Output
```bash
ros2 topic echo /illuminance
```

**Output Example:**
```yaml
header:
  stamp:
    sec: 1709210000
    nanosec: 123456789
  frame_id: bh1750_link
illuminance: 125.4
variance: 0.0
```

## ⚙️ Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `i2c_bus` | int | `1` | I2C Bus ID (usually 1 on RPi) |
| `i2c_address` | int | `0x23` | I2C Address (`0x23` or `0x5C`) |
| `publish_rate` | double | `1.0` | Publishing frequency in Hz |
| `frame_id` | string | `bh1750_link` | TF Frame ID |
| `mode` | string | `CONTINUOUS_HIGH_RES_MODE` | Measurement mode |

**Available Modes:**
*   `CONTINUOUS_HIGH_RES_MODE` (1 lx res, ~120ms)
*   `CONTINUOUS_HIGH_RES_MODE_2` (0.5 lx res, ~120ms)
*   `CONTINUOUS_LOW_RES_MODE` (4 lx res, ~16ms)
*   `ONE_TIME_HIGH_RES_MODE`
*   `ONE_TIME_HIGH_RES_MODE_2`
*   `ONE_TIME_LOW_RES_MODE`

## 🛠 Troubleshooting
*   **`IOError: [Errno 121] Remote I/O error`**: Check wiring. Ensure I2C is enabled (`sudo raspi-config`).
*   **Reading is 0 or constant**: Sensor might be saturated (too bright) or disconnected.
*   **Wrong Address**: Try running `i2cdetect -y 1` to find the correct address.
