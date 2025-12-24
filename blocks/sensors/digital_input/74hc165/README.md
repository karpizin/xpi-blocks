# Input Expander: 74HC165 Shift Register

The 74HC165 is an 8-bit parallel-in, serial-out shift register. It allows you to read 8 digital inputs (like buttons or switches) using only 3 GPIO pins on the Raspberry Pi.

## 📌 Features
*   **Expansion:** 8 inputs per chip.
*   **Daisy-chainable:** Supports connecting multiple chips in series.
*   **Interface:** GPIO (Data, Clock, Load).

## 🔌 Wiring Diagram

| 74HC165 Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VCC (16) | 3.3V | Red | |
| GND (8) | GND | Black | |
| PL (1) | GPIO 23 (Pin 16) | Green | Parallel Load (Active LOW) |
| CP (2) | GPIO 24 (Pin 18) | Yellow | Clock |
| Q7 (9) | GPIO 25 (Pin 22) | Blue | Serial Data Out |
| CE (15) | GND | - | Clock Enable (Active LOW) |

## 🚀 Quick Start

1.  **Run the Node:**
    ```bash
    ros2 launch xpi_sensors shift_register_165.launch.py
    ```
2.  **Read Input State:**
    ```bash
    ros2 topic echo /shift_register_165/raw_mask
    ```

## 📊 Published Topics
*   `~/raw_mask` ([std_msgs/Int32](http://docs.ros.org/en/api/std_msgs/html/msg/Int32.html)): The current 8-bit state of the inputs.
