# Movement: PIR Motion Sensor (GPIO)

Passive Infrared (PIR) sensors are electronic sensors that measure infrared light radiating from objects in their field of view. They are most often used in PIR-based motion detectors.

## 📌 Features
*   **Detection Angle:** ~100-110°.
*   **Range:** Up to 7 meters (adjustable).
*   **Interface:** Digital GPIO (HIGH on motion).
*   **Compatibility:** Works with HC-SR501, AM312, and similar modules.

## 🔌 Wiring Diagram

| PIR Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VCC | 5V (Pin 2) | Red | HC-SR501 usually requires 5V |
| GND | GND (Pin 6) | Black | |
| OUT | GPIO 26 (Pin 37) | Yellow | Logic HIGH (3.3V) when motion is detected |

> **Hardware Adjustment:** For HC-SR501, use the onboard potentiometers to adjust **Sensitivity** (distance) and **Time Delay** (how long it stays HIGH).

## 🚀 Quick Start

1.  **Run the Node:**
    ```bash
    ros2 launch xpi_sensors pir.launch.py
    ```
2.  **Monitor Motion:**
    ```bash
    ros2 topic echo /pir/motion
    ```

## 📊 Published Topics
*   `~/motion` ([std_msgs/Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html)): `true` if motion is detected, `false` otherwise.
