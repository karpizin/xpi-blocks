# Environment: AHT10 / AHT20 (I2C)

AHT10 and AHT20 are low-cost, high-precision temperature and humidity sensors. They are modern alternatives to the DHT series, providing a digital I2C interface for easier and more reliable integration.

## 📌 Features
*   **AHT10:** 1.8V to 3.6V, 2% accuracy for humidity, 0.3°C for temperature.
*   **AHT20:** Improved version, 2.0V to 5.5V, more stable in harsh environments.
*   **Interface:** I2C.

## 🔌 Wiring Diagram

| AHT20 Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VIN | 3.3V (Pin 1) | Red | |
| GND | GND (Pin 9) | Black | |
| SCL | SCL (GPIO 3 / Pin 5) | Yellow | |
| SDA | SDA (GPIO 2 / Pin 3) | Blue | |

## 🚀 Quick Start

1.  **Enable I2C:** Use `raspi-config` to enable the I2C interface.
2.  **Run the Node:**
    ```bash
    ros2 launch xpi_sensors aht20.launch.py
    ```
3.  **Monitor Data:**
    ```bash
    ros2 topic echo /aht20/temperature
    ros2 topic echo /aht20/humidity
    ```

## 📊 Published Topics
*   `~/temperature` ([sensor_msgs/Temperature](http://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html)): Ambient temperature in Celsius.
*   `~/humidity` ([sensor_msgs/RelativeHumidity](http://docs.ros.org/en/api/sensor_msgs/html/msg/RelativeHumidity.html)): Relative humidity (0.0 to 1.0).
