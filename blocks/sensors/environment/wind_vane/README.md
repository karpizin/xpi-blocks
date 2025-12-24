# Weather Station: Wind Vane (Analog)

The Wind Vane sensor determines the direction of the wind. Most affordable wind vanes (like those from Misol, Fine Offset, or SparkFun) use a magnetic reed switch and a resistor network.

## 📌 Features
*   **16 Directions:** Detects 16 cardinal and intercardinal directions.
*   **Interface:** Analog (requires an ADC like ADS1115).
*   **Output:** Direction in degrees (0-359°) and cardinal points (N, NE, E, etc.).

## 🔌 Wiring Diagram

To use this with a Raspberry Pi, connect the Wind Vane to an **ADS1115 ADC** using a voltage divider or a simple pull-up resistor.

### Recommended Circuit (Pull-up)
1. Connect one wire of the Wind Vane to **GND**.
2. Connect the other wire to **ADS1115 Channel 0 (A0)**.
3. Connect a **10k Ohm resistor** between **A0** and **3.3V**.

| Component | Connect to |
| :--- | :--- |
| Wind Vane Pin 1 | GND |
| Wind Vane Pin 2 | ADS1115 A0 |
| Pull-up Resistor (10k) | Between A0 and 3.3V |

## 🚀 Quick Start

1.  **Launch ADS1115:**
    ```bash
    ros2 launch xpi_sensors ads1115.launch.py channels:="[0]"
    ```
2.  **Launch Wind Vane Node:**
    ```bash
    ros2 launch xpi_sensors wind_vane.launch.py
    ```
3.  **Monitor Direction:**
    ```bash
    ros2 topic echo /wind_vane/direction
    ```

## 📊 Published Topics
*   `~/direction` ([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html)): Wind direction in degrees (0 = North).
*   `~/cardinal` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)): Wind direction as text (e.g., "NE", "SSW").
