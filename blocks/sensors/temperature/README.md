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
