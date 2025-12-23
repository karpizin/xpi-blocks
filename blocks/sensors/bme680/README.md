# BME680 Environmental Sensor (Gas, Temp, Hum, Press)

The **BME680** is an integrated environmental sensor developed specifically for mobile applications and wearables where size and low power consumption are key requirements. It measures **gas quality (VOC)**, **temperature**, **pressure**, and **humidity**.

This block uses the `adafruit-circuitpython-bme680` library via `Adafruit Blinka` compatibility layer for ROS2.

## 📦 Bill of Materials
*   Raspberry Pi (3B+, 4, 5, or Zero 2W)
*   BME680 Sensor Module (I2C) - [Adafruit](https://www.adafruit.com/product/3660) or generic.
*   Jumper Wires (Female-to-Female)

## 🔌 Wiring
Connect the BME680 module to the I2C pins on the Raspberry Pi.

| BME680 Pin | Raspberry Pi | Note |
|------------|--------------|-----------------------------------|
| VIN/VCC    | 3.3V         | Power (Pin 1 or 17)               |
| GND        | GND          | Ground (Pin 6, 9, etc.)           |
| SDA        | GPIO 2 (SDA) | I2C Data (Pin 3)                  |
| SCL        | GPIO 3 (SCL) | I2C Clock (Pin 5)                 |

**Note on Addresses:**
*   Adafruit modules usually default to **0x77**.
*   Generic Chinese modules often default to **0x76** or have a solder bridge to select.
*   You can verify using `i2cdetect -y 1`.

## 🛠 Software Setup

1.  **Enable I2C:**
    ```bash
    sudo raspi-config
    # Interface Options -> I2C -> Yes
    ```

2.  **Install System Dependencies:**
    This node requires `libgpiod` for CircuitPython Blinka:
    ```bash
    sudo apt-get install python3-libgpiod
    ```

3.  **Install Python Dependencies:**
    (Already included in project `requirements.txt`)
    ```bash
    pip3 install adafruit-circuitpython-bme680
    ```

## 🚀 Usage

**Launch the node:**
```bash
ros2 launch xpi_sensors bme680.launch.py
```

**With custom address (e.g., 0x76):**
```bash
ros2 launch xpi_sensors bme680.launch.py i2c_address:=0x76
```

## 📡 Interface

### Publishers
*   `~/temperature` (`sensor_msgs/Temperature`): Ambient temperature in Celsius.
*   `~/humidity` (`sensor_msgs/RelativeHumidity`): Relative humidity (0.0 to 1.0).
*   `~/pressure` (`sensor_msgs/FluidPressure`): Atmospheric pressure in Pascals.
*   `~/gas_resistance` (`std_msgs/Float32`): Gas resistance in Ohms.
    *   *Note:* Higher gas resistance = Cleaner air. Lower resistance = More VOCs present.
    *   The sensor needs a "burn-in" period (20-30 mins) for gas readings to stabilize initially.

### Parameters
*   `i2c_address` (int, default: `0x77`): I2C address of the sensor.
*   `publish_rate` (float, default: `1.0`): Frequency to publish data in Hz.
*   `frame_id` (string, default: `bme680_link`): Frame ID for TF tree.
*   `sea_level_pressure` (float, default: `1013.25`): Used internally for altitude calculation if needed.

## ✅ Verification

1.  Start the node.
2.  Echo the topics:
    ```bash
    ros2 topic echo /bme680_env_sensor/gas_resistance
    ```
3.  Breathe gently near the sensor (alcohol/VOCs from breath) -> Resistance should **drop**.
4.  Ventilate the area -> Resistance should **rise**.

## ⚠️ Troubleshooting
*   **"ValueError: No I2C device at address..."**:
    *   Wrong address. Try `0x76` instead of `0x77`.
    *   Check wiring.
    *   Run `i2cdetect -y 1` to confirm visibility.
*   **"Gas resistance is 0 or unstable"**:
    *   The heater needs time to warm up. Ignore the first few readings.
    *   Ensure the power supply is stable (heater consumes current).
