# BME280 Environmental Sensor (Temperature, Humidity, Pressure)

This block provides a ROS2 driver for the BME280, a popular digital sensor for measuring atmospheric pressure, ambient temperature, and relative humidity. It publishes `sensor_msgs/Temperature`, `sensor_msgs/RelativeHumidity`, and `sensor_msgs/FluidPressure` messages.

## 📦 Bill of Materials
*   Raspberry Pi
*   BME280 module (I2C interface)
*   Jumper Wires

## 🔌 Wiring
Connect the BME280 module to the I2C pins on the Raspberry Pi.

| BME280 Pin | Raspberry Pi | Note |
|------------|--------------|-----------------------------------|
| VCC        | 3.3V         | Power for the module (Pin 1)      |
| GND        | GND          | Common Ground (Pin 6)             |
| SDA        | GPIO 2 (SDA) | Data line (Pin 3)                 |
| SCL        | GPIO 3 (SCL) | Clock line (Pin 5)                |

**Important I2C Configuration on Raspberry Pi:**
You must enable the I2C interface on your Raspberry Pi.

1.  **Enable I2C:**
    ```bash
    sudo raspi-config
    # Interface Options -> P3 I2C -> Yes
    ```
2.  **Verify I2C Address:** Check the BME280's I2C address (usually 0x76 or 0x77)
    ```bash
    sudo apt install i2c-tools
    i2cdetect -y 1 # Or 0 if using an older Pi
    # Look for '76' or '77' in the output.
    ```

## 🚀 Quick Start
1.  **Perform I2C Configuration** as described above.
2.  **Launch the BME280 driver**:
    ```bash
    ros2 launch xpi_sensors bme280.launch.py i2c_address:=0x76
    ```
    (Adjust `i2c_address` if yours is 0x77, or other parameters as needed).

## 📡 Interface
### Publishers
*   `~/temperature` (`sensor_msgs/Temperature`): Publishes ambient temperature in Celsius.
*   `~/humidity` (`sensor_msgs/RelativeHumidity`): Publishes relative humidity (0.0 to 1.0).
*   `~/pressure` (`sensor_msgs/FluidPressure`): Publishes atmospheric pressure in Pascals.

### Parameters
*   `i2c_bus` (int, default: `1`): I2C bus number.
*   `i2c_address` (int, default: `0x76`): I2C address of the BME280.
*   `publish_rate` (float, default: `1.0`): Frequency to publish data in Hz.
*   `osrs_t` (int, default: `2`): Oversampling for temperature (0:skipped, 1:1x, 2:2x, 3:4x, 4:8x, 5:16x).
*   `osrs_p` (int, default: `5`): Oversampling for pressure (0:skipped, 1:1x, 2:2x, 3:4x, 4:8x, 5:16x).
*   `osrs_h` (int, default: `1`): Oversampling for humidity (0:skipped, 1:1x, 2:2x, 3:4x, 4:8x, 5:16x).
*   `mode` (int, default: `3`): Power mode (0:sleep, 1:forced, 3:normal).
*   `frame_id` (string, default: `bme280_link`): Frame ID for messages.
*   `mock_hardware` (bool, default: `false`): Run in mock mode without real BME280 hardware.

## ✅ Verification
1.  Launch the driver with your BME280 connected and I2C enabled.
2.  In new terminals, monitor the published topics:
    ```bash
    ros2 topic echo /bme280_env_sensor/temperature
    ros2 topic echo /bme280_env_sensor/humidity
    ros2 topic echo /bme280_env_sensor/pressure
    ```
    You should see streams of environmental data.

## ⚠️ Troubleshooting
*   **"Failed to initialize BME280" / "I/O Error"**:
    *   Double-check I2C wiring (SDA/SCL, VCC/GND).
    *   Verify I2C is enabled (`sudo raspi-config`).
    *   Ensure the correct `i2c_address` is used.
    *   Check for I2C permissions (`sudo usermod -a -G i2c $USER`).
*   **No data / erratic data**:
    *   Check physical connections.
    *   Verify module is powered.
    *   Ensure proper grounding.
