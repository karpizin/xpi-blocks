# Weather Station: Digital Wind Vane (AS5600 I2C)

This digital wind vane uses the **AS5600 absolute magnetic encoder**. It provides high-precision (12-bit) direction sensing without any mechanical contact points or dead zones.

## 📌 Features
*   **High Precision:** 4096 positions per 360° (0.087° resolution).
*   **Magnetic Sensing:** No friction from wipers (unlike analog potentiometers).
*   **Interface:** I2C.
*   **Output:** Direction in degrees (0-359°) and cardinal points.

## 🔌 Wiring Diagram

| AS5600 Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VCC | 3.3V (Pin 1) | Red | |
| GND | GND (Pin 9) | Black | |
| SCL | SCL (GPIO 3 / Pin 5) | Yellow | |
| SDA | SDA (GPIO 2 / Pin 3) | Blue | |
| PGO | - | - | Not used |

> **Hardware Note:** Ensure the magnet is correctly positioned above the AS5600 chip (typically 0.5mm - 3mm gap).

## 🚀 Quick Start

1.  **Enable I2C:** Use `raspi-config` to enable the I2C interface.
2.  **Run the Node:**
    ```bash
    ros2 launch xpi_sensors wind_vane_digital.launch.py
    ```
3.  **Monitor Direction:**
    ```bash
    ros2 topic echo /wind_vane_digital/direction
    ```

## 📊 Published Topics
*   `~/direction` ([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html)): Wind direction in degrees.
*   `~/cardinal` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)): Cardinal direction (N, NE, E, etc.).
*   `~/status` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)): Magnetic sensor status (e.g., "Magnet OK", "Magnet too weak").
