# Environment: SHT30 / SHT31 / SHT35 (I2C)

The SHT3x series are high-precision, digital temperature and humidity sensors. They offer superior accuracy, fast response times, and high reliability in various environmental conditions.

## 📌 Features
*   **SHT30:** ±2% RH accuracy, ±0.2°C temperature accuracy (standard).
*   **SHT31:** ±2% RH accuracy, ±0.2°C temperature accuracy (high precision).
*   **SHT35:** ±1.5% RH accuracy, ±0.1°C temperature accuracy (high-end).
*   **Interface:** I2C (Address 0x44 or 0x45).
*   **Built-in Heater:** For clearing condensation.

## 🔌 Wiring Diagram

| SHT3x Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VIN | 3.3V (Pin 1) | Red | |
| GND | GND (Pin 9) | Black | |
| SCL | SCL (GPIO 3 / Pin 5) | Yellow | |
| SDA | SDA (GPIO 2 / Pin 3) | Blue | |
| ADDR | - | - | Connect to VIN for 0x45 address |

## 🚀 Quick Start

1.  **Enable I2C:** Use `raspi-config` to enable the I2C interface.
2.  **Run the Node:**
    ```bash
    ros2 launch xpi_sensors sht3x.launch.py
    ```
3.  **Monitor Data:**
    ```bash
    ros2 topic echo /sht3x/temperature
    ros2 topic echo /sht3x/humidity
    ```

## 📊 Published Topics
*   `~/temperature` ([sensor_msgs/Temperature](http://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html)): Ambient temperature in Celsius.
*   `~/humidity` ([sensor_msgs/RelativeHumidity](http://docs.ros.org/en/api/sensor_msgs/html/msg/RelativeHumidity.html)): Relative humidity (0.0 to 1.0).
