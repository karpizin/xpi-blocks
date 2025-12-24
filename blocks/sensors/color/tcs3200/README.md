# Color Sensor: TCS3200 (GY-31)

The TCS3200 is a color-to-frequency converter that combines configurable silicon photodiodes and a current-to-frequency converter on a single monolithic CMOS integrated circuit.

## 📌 Features
*   **RGB + Clear:** Four types of photodiode filters.
*   **Programmable Frequency Scaling:** 100%, 20%, or 2%.
*   **Output:** Square wave with frequency proportional to light intensity.
*   **Interface:** Digital GPIO (S0-S3, OUT).

## 🔌 Wiring Diagram

| TCS3200 Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VCC | 3.3V or 5V | Red | |
| GND | GND | Black | |
| S0 | GPIO 23 (Pin 16) | White | Frequency Scaling |
| S1 | GPIO 24 (Pin 18) | Grey | Frequency Scaling |
| S2 | GPIO 25 (Pin 22) | Yellow | Photodiode Select |
| S3 | GPIO 8  (Pin 24) | Blue | Photodiode Select |
| OUT | GPIO 7  (Pin 26) | Green | Signal Out |
| LED | GPIO 12 (Pin 32) | Orange | Onboard LEDs Control |

### Logic Table (Photodiode Select)
| S2 | S3 | Photodiode Type |
| :--- | :--- | :--- |
| L | L | Red |
| L | H | Blue |
| H | L | Clear (no filter) |
| H | H | Green |

## 🚀 Quick Start

1.  **Run the Node:**
    ```bash
    ros2 launch xpi_sensors tcs3200.launch.py
    ```
2.  **Monitor Color Data:**
    ```bash
    ros2 topic echo /tcs3200/color
    ```

## 📊 Published Topics
*   `~/color` ([std_msgs/ColorRGBA](http://docs.ros.org/en/api/std_msgs/html/msg/ColorRGBA.html)): Normalized RGB values (0.0 - 1.0).
*   `~/raw_frequencies` ([std_msgs/Int32MultiArray](http://docs.ros.org/en/api/std_msgs/html/msg/Int32MultiArray.html)): Raw pulse counts for R, G, B, and Clear channels.
