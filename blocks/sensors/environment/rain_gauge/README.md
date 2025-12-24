# Weather Station: Rain Gauge (Pulse Counter)

The Rain Gauge measures the amount of liquid precipitation. The most common type is the "tipping bucket" gauge, which uses a small seesaw mechanism. Each "tip" corresponds to a fixed amount of rainfall.

## 📌 Features
*   **Total Rainfall:** Tracks cumulative precipitation in mm.
*   **Rain Intensity:** Calculates current rainfall rate (mm/hour).
*   **Interface:** Digital (Pulse/GPIO).

## 🔌 Wiring Diagram

| Rain Gauge Wire | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| Wire 1 | GPIO 22 (Pin 15) | Green | Connected to Reed Switch |
| Wire 2 | GND (Pin 14) | Black | |

> **Note:** Tipping bucket gauges are purely mechanical switches (reed switches). They don't require power, just a pull-up resistor (internally enabled in our node).

## 🚀 Quick Start

1.  **Launch the Node:**
    ```bash
    ros2 launch xpi_sensors rain_gauge.launch.py
    ```
2.  **Monitor Rainfall:**
    ```bash
    ros2 topic echo /rain_gauge/total
    ros2 topic echo /rain_gauge/intensity
    ```

## 📊 Published Topics
*   `~/total` ([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html)): Total cumulative rainfall in mm.
*   `~/intensity` ([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html)): Current rainfall rate in mm/hour.
