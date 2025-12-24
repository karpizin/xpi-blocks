# Range: TFmini Plus LiDAR (Outdoor/IP65)

The TFmini Plus is a mini LiDAR module based on Time-of-Flight (ToF) technology. It is designed for outdoor use with an IP65 rated housing and improved compensation for ambient light.

## 📌 Features
*   **Operating Range:** 0.1m - 12m.
*   **Frame Rate:** Up to 1000Hz (default 100Hz).
*   **Enclosure:** IP65 (Water and dust resistant).
*   **Interface:** UART (Serial) / I2C.
*   **Accuracy:** ±5cm @ 6m, ±1% @ 12m.

## 🔌 Wiring Diagram (UART)

| TFmini Plus Pin | Raspberry Pi Pin | Color | Note |
| :--- | :--- | :--- | :--- |
| +5V | 5V (Pin 2/4) | Red | Requires 5V power |
| GND | GND (Pin 6) | Black | |
| TX | RXD (GPIO 15 / Pin 10) | Green | TFmini TX -> RPi RX |
| RX | TXD (GPIO 14 / Pin 8) | White | TFmini RX -> RPi TX |

> **Note:** Remember to disable the Serial Console in `raspi-config` before using UART for sensors.

## 🚀 Quick Start

1.  **Launch the Node:**
    ```bash
    ros2 launch xpi_sensors tfmini_plus.launch.py port:="/dev/ttyS0"
    ```
2.  **Monitor Range:**
    ```bash
    ros2 topic echo /tfmini_plus/range
    ```

## 📊 Published Topics
*   `~/range` ([sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)): Distance in meters.
*   `~/strength` ([std_msgs/Int32](http://docs.ros.org/en/api/std_msgs/html/msg/Int32.html)): Signal strength (useful for reliability checks).
