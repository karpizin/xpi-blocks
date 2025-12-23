# TSL2591 High Dynamic Range Light Sensor

The **TSL2591** is a very high sensitivity light-to-digital converter that transforms light intensity into a digital signal output capable of direct I2C interface. It has a huge dynamic range (188uLux to 88,000 Lux).

## 📦 Bill of Materials
*   Raspberry Pi
*   TSL2591 Module (I2C) - [Adafruit](https://www.adafruit.com/product/1980) or generic.
*   Jumper Wires

## 🔌 Wiring
Connect the TSL2591 module to the I2C pins.

| TSL2591 Pin | Raspberry Pi | Note |
|-------------|--------------|-----------------------------------|
| VIN         | 3.3V or 5V   | Power (Pin 1).                    |
| GND         | GND          | Ground (Pin 6)                    |
| SDA         | GPIO 2 (SDA) | Data (Pin 3)                      |
| SCL         | GPIO 3 (SCL) | Clock (Pin 5)                     |

**I2C Address:** 0x29 (Fixed).

## 🛠 Software Setup

1.  **Install System Dependencies:**
    ```bash
    sudo apt-get install python3-libgpiod
    ```

2.  **Install Python Library:**
    ```bash
    pip3 install adafruit-circuitpython-tsl2591
    ```

## 🚀 Usage

**Launch with defaults:**
```bash
ros2 launch xpi_sensors tsl2591.launch.py
```

**Launch with High Gain (for low light):**
```bash
ros2 launch xpi_sensors tsl2591.launch.py gain:=HIGH
```

## 📡 Interface

### Publishers
*   `~/illuminance` (`sensor_msgs/Illuminance`): Calculated Lux.
*   `~/infrared_raw` (`std_msgs/Float32`): Raw infrared channel count.

### Parameters
*   `polling_rate` (float, default: `2.0`): Hz.
*   `gain` (string, default: `MEDIUM`):
    *   `LOW`: 1x (Bright sun)
    *   `MEDIUM`: 25x (General indoor)
    *   `HIGH`: 428x (Low light)
    *   `MAX`: 9876x (Extremely low light)
*   `integration_time` (string, default: `100MS`): `100MS` to `600MS`. Longer time = more light gathered but slower update.
*   `frame_id` (string, default: `tsl2591_link`): Frame ID for TF.

## ⚠️ Troubleshooting
*   **"Lux 0.0"**: 
    *   If it's too bright for the current gain settings, the sensor might saturate. Try lowering gain to `LOW`.
    *   If it's pitch black, this is correct.
