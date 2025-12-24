# Output Expander: 74HC595 Shift Register

The 74HC595 is an 8-bit serial-in, parallel-out shift register. It allows you to control 8 digital outputs using only 3 GPIO pins on the Raspberry Pi. You can also daisy-chain multiple registers to control even more outputs.

## 📌 Features
*   **Expansion:** 8 outputs per chip.
*   **Daisy-chainable:** Connect the 'Serial Out' of one to the 'Serial In' of another.
*   **Interface:** GPIO (Data, Clock, Latch).

## 🔌 Wiring Diagram

| 74HC595 Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VCC (16) | 3.3V or 5V | Red | |
| GND (8) | GND | Black | |
| DS (14) | GPIO 17 (Pin 11) | Blue | Serial Data Input |
| SH_CP (11) | GPIO 27 (Pin 13) | Yellow | Shift Register Clock |
| ST_CP (12) | GPIO 22 (Pin 15) | Green | Storage Register Clock (Latch) |
| OE (13) | GND | - | Output Enable (Active LOW) |
| MR (10) | VCC | - | Master Reset (Active LOW) |

## 🚀 Quick Start

1.  **Run the Node:**
    ```bash
    ros2 launch xpi_actuators shift_register_595.launch.py
    ```
2.  **Set Output State (Binary Mask):**
    ```bash
    # Turn on outputs 0 and 2 (binary 00000101 = decimal 5)
    ros2 topic pub --once /shift_register_595/raw_mask std_msgs/msg/Int32 "{data: 5}"
    ```

## 📊 Topics
*   `~/raw_mask` ([std_msgs/Int32](http://docs.ros.org/en/api/std_msgs/html/msg/Int32.html)): Set the 8-bit state of the register.
