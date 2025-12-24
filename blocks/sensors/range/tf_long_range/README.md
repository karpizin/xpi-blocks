# Range: TF02-Pro & TF03-100 Long Range LiDAR (UART)

The TF02-Pro and TF03-100 are high-performance LiDAR rangefinders designed for industrial and outdoor applications. They offer high stability, high precision, and a very long range.

## 📌 Features
*   **TF02-Pro:** Range up to 40 meters. Optimized for outdoor stability.
*   **TF03-100:** Range up to 100 meters (or 180m for the high-end version). High industrial grade.
*   **Enclosure:** IP65 / IP67 rated.
*   **Interface:** UART (Serial) / CAN.

## 🔌 Wiring Diagram

| LiDAR Pin | Raspberry Pi Pin | Color | Note |
| :--- | :--- | :--- | :--- |
| +5V | 5V (Pin 2/4) | Red | |
| GND | GND (Pin 6) | Black | |
| TX | RXD (GPIO 15 / Pin 10) | Green | Sensor TX -> RPi RX |
| RX | TXD (GPIO 14 / Pin 8) | White | Sensor RX -> RPi TX |

## 🚀 Quick Start

1.  **Launch the Node:**
    ```bash
    ros2 launch xpi_sensors tf_lidar.launch.py model:="tf02_pro"
    ```
2.  **Monitor Data:**
    ```bash
    ros2 topic echo /tf_lidar/range
    ```

## 📊 Published Topics
*   `~/range` ([sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)): Distance in meters.
*   `~/strength` ([std_msgs/Int32](http://docs.ros.org/en/api/std_msgs/html/msg/Int32.html)): Signal strength indicator.
*   `~/temp` ([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html)): Internal sensor temperature (if supported).
