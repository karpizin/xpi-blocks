# Environment: SCD40 / SCD41 CO2 Sensor (I2C)

The SCD4x series from Sensirion are high-quality, true CO2 sensors based on the photoacoustic sensing principle. They provide accurate measurement of CO2 concentration, temperature, and humidity.

## 📌 Features
*   **True CO2 Sensing:** NDIR-equivalent accuracy in a tiny form factor.
*   **Range:** 400 – 2000 ppm (SCD40) / 400 – 5000 ppm (SCD41).
*   **Output:** CO2 (ppm), Temperature (°C), and Relative Humidity (%).
*   **Interface:** I2C (Address 0x62).

## 🔌 Wiring Diagram

| SCD4x Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VCC | 3.3V or 5V | Red | 2.4V - 5.5V supported |
| GND | GND | Black | |
| SCL | SCL (GPIO 3) | Yellow | |
| SDA | SDA (GPIO 2) | Blue | |

## 🚀 Quick Start

1.  **Enable I2C:** Use `raspi-config` to enable the I2C interface.
2.  **Run the Node:**
    ```bash
    ros2 launch xpi_sensors scd4x.launch.py
    ```
3.  **Monitor Data:**
    ```bash
    ros2 topic echo /scd4x/co2
    ros2 topic echo /scd4x/temperature
    ```

## 📊 Published Topics
*   `~/co2` ([std_msgs/Int32](http://docs.ros.org/en/api/std_msgs/html/msg/Int32.html)): CO2 concentration in ppm.
*   `~/temperature` ([sensor_msgs/Temperature](http://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html)): Ambient temperature.
*   `~/humidity` ([sensor_msgs/RelativeHumidity](http://docs.ros.org/en/api/sensor_msgs/html/msg/RelativeHumidity.html)): Relative humidity (0.0 to 1.0).
