# DS18B20 1-Wire Temperature Sensor

This block provides a ROS2 driver for reading temperature from DS18B20 1-Wire sensors. It publishes the temperature as `sensor_msgs/Temperature` messages.

## 📦 Bill of Materials
*   Raspberry Pi
*   DS18B20 1-Wire Temperature Sensor
*   4.7kΩ Resistor (for pull-up)
*   Jumper Wires

## 🔌 Wiring
Connect the DS18B20 to a GPIO pin on the Raspberry Pi. The data pin of the DS18B20 requires a 4.7kΩ pull-up resistor to 3.3V.

| DS18B20 Pin | Raspberry Pi | Note |
|-------------|--------------|-----------------------------------|
| VCC         | 3.3V         | Power for the sensor (Pin 1)      |
| GND         | GND          | Common Ground (Pin 6)             |
| DATA        | GPIO 4       | Data pin. **Requires 4.7kΩ pull-up to 3.3V!** (Pin 7) |

**Important 1-Wire Configuration on Raspberry Pi:**
You must enable the 1-Wire interface on your Raspberry Pi.

1.  **Enable 1-Wire:**
    ```bash
    sudo raspi-config
    # Interface Options -> P3 1-Wire -> Yes
    ```
2.  **Reboot:**
    ```bash
    sudo reboot
    ```
3.  **Verify:** After reboot, check for 1-Wire devices:
    ```bash
    ls /sys/bus/w1/devices/
    # You should see folders like '28-00000xxxxxxx'
    ```

## 🚀 Quick Start
1.  **Perform 1-Wire Configuration** as described above.
2.  **Launch the DS18B20 driver**:
    ```bash
    ros2 launch xpi_sensors ds18b20.launch.py device_id:=28-00000xxxxxxx # Use your sensor's actual ID
    ```
    If you only have one sensor, you can often use the default `device_id:=28-*` to find the first available sensor.

## 📡 Interface
### Publishers
*   `~/temperature` (`sensor_msgs/Temperature`): Publishes temperature in Celsius.

### Parameters
*   `device_id` (string, default: `28-*`): The 1-Wire ID of your DS18B20 sensor. Can be a wildcard `28-*` to find the first one.
*   `publish_rate` (float, default: `1.0`): Frequency to publish temperature readings in Hz.
*   `frame_id` (string, default: `ds18b20_link`): Frame ID for the `sensor_msgs/Temperature` message.
*   `mock_hardware` (bool, default: `false`): Run in mock mode for testing.

## ✅ Verification
1.  Launch the driver with your DS18B20 connected and 1-Wire enabled.
2.  In a new terminal, monitor the `/ds18b20_temp_sensor/temperature` topic:
    ```bash
    ros2 topic echo /ds18b20_temp_sensor/temperature
    ```
    You should see temperature readings.

## ⚠️ Troubleshooting
*   **"Device not found" error:**
    *   Double-check wiring, especially the 4.7kΩ pull-up resistor.
    *   Verify 1-Wire is enabled (`sudo raspi-config`).
    *   Check `ls /sys/bus/w1/devices/` to find your sensor's ID and use it in `device_id` parameter.
*   **No temperature readings:**
    *   Ensure the sensor is correctly connected and powered.
    *   Check for errors in the node's logs.

---

# LM75A Digital Temperature Sensor

This block provides a ROS2 driver for the LM75A, a simple digital temperature sensor with a thermostat function, communicating via I2C. It publishes `sensor_msgs/Temperature` messages.

## 📦 Bill of Materials
*   Raspberry Pi
*   LM75A module (I2C interface)
*   Jumper Wires

## 🔌 Wiring
Connect the LM75A module to the I2C pins on the Raspberry Pi.

| LM75A Pin | Raspberry Pi | Note |
|-----------|--------------|-----------------------------------|
| VCC       | 3.3V         | Power for the module (Pin 1)      |
| GND       | GND          | Common Ground (Pin 6)             |
| SDA       | GPIO 2 (SDA) | Data line (Pin 3)                 |
| SCL       | GPIO 3 (SCL) | Clock line (Pin 5)                |
| A0, A1, A2| GND/3.3V     | Sets I2C address (e.g., all to GND for 0x48) |

**Important I2C Configuration on Raspberry Pi:**
You must enable the I2C interface on your Raspberry Pi.

1.  **Enable I2C:**
    ```bash
    sudo raspi-config
    # Interface Options -> P3 I2C -> Yes
    ```
2.  **Verify I2C Address:** Check the LM75A's I2C address (e.g., 0x48 if A0, A1, A2 are all connected to GND)
    ```bash
    sudo apt install i2c-tools
    i2cdetect -y 1 # Or 0 if using an older Pi
    # Look for '48' (or your configured address) in the output.
    ```

## 🚀 Quick Start
1.  **Perform I2C Configuration** as described above.
2.  **Launch the LM75A driver**:
    ```bash
    ros2 launch xpi_sensors lm75a.launch.py i2c_address:=0x48
    ```
    (Adjust `i2c_address` if necessary).

## 📡 Interface
### Publishers
*   `~/temperature` (`sensor_msgs/Temperature`): Publishes ambient temperature in Celsius.

### Parameters
*   `i2c_bus` (int, default: `1`): I2C bus number.
*   `i2c_address` (int, default: `0x48`): I2C address of the LM75A.
*   `publish_rate` (float, default: `1.0`): Frequency to publish data in Hz.
*   `frame_id` (string, default: `lm75a_link`): Frame ID for messages.
*   `mock_hardware` (bool, default: `false`): Run in mock mode without real LM75A hardware.

## ✅ Verification
1.  Launch the driver with your LM75A connected and I2C enabled.
2.  In a new terminal, monitor the `/lm75a_temp_sensor/temperature` topic:
    ```bash
    ros2 topic echo /lm75a_temp_sensor/temperature
    ```
    You should see temperature readings.

## ⚠️ Troubleshooting
*   **"Failed to initialize LM75A" / "I/O Error"**:
    *   Double-check I2C wiring (SDA/SCL, VCC/GND).
    *   Verify I2C is enabled (`sudo raspi-config`).
    *   Ensure the correct `i2c_address` is used.
    *   Check for I2C permissions (`sudo usermod -a -G i2c $USER`).
*   **No data / erratic data**:
    *   Check physical connections.
    *   Verify module is powered.
    *   Ensure proper grounding.
