# TSL2561 Luminosity Sensor

The TSL2561 is a sophisticated light sensor that converts light intensity to a digital signal output. It has two photodiodes: one for the broad spectrum (visible + IR) and one for infrared only. This allows for precise calculation of visible light lux, filtering out IR noise.

![TSL2561](https://cdn-shop.adafruit.com/970x728/439-00.jpg)
*(Image is illustrative, generic TSL2561 module)*

## ⚡ Wiring

The TSL2561 communicates via I2C.

| TSL2561 Pin | Raspberry Pi Pin | Description |
| :--- | :--- | :--- |
| **VCC** | 3.3V (Pin 1 or 17) | Power (2.7V - 3.6V) |
| **GND** | GND (Pin 6, 9, etc) | Ground |
| **SDA** | GPIO 2 (Pin 3) | I2C Data |
| **SCL** | GPIO 3 (Pin 5) | I2C Clock |
| **ADDR** | (Optional) | Selects I2C Address (See below) |

**Address Selection (ADDR Pin):**
*   **Float (Unconnected):** 0x39 (Default)
*   **GND:** 0x29
*   **VCC:** 0x49

## 📦 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `i2c_bus` | int | `1` | I2C bus number |
| `i2c_address` | int | `0x39` | I2C address |
| `poll_rate` | float | `1.0` | Read frequency in Hz |
| `gain_16x` | bool | `False` | `True` for 16x gain (Low light), `False` for 1x (Bright light) |
| `integration_time` | int | `2` | `0`=13.7ms, `1`=101ms, `2`=402ms (Default) |

## 🚀 Usage

**Run the node:**
```bash
ros2 launch xpi_sensors tsl2561.launch.py
```

**Verify output:**
```bash
ros2 topic echo /tsl2561/illuminance
```

Output should look like:
```yaml
header:
  stamp: ...
  frame_id: tsl2561_link
illuminance: 320.5
variance: 0.0
```

## 📝 Notes
*   This driver implements the standard "Lux Calculation" algorithm from the datasheet (T, FN, CL packages).
*   It automatically normalizes readings based on the selected `gain` and `integration_time`, so the output Lux should be accurate regardless of settings (unless the sensor is saturated).
*   If you see `0.0` lux in very bright light, try setting `gain_16x` to `False`.
*   If you see `0.0` lux in pitch black, that's correct :)
