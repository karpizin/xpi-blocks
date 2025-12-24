# Weight: HX711 24-bit ADC for Load Cells

The HX711 is a precision 24-bit analog-to-digital converter (ADC) designed for weigh scales and industrial control applications to interface directly with a bridge sensor.

## 📌 Features
*   **High Precision:** 24-bit resolution.
*   **Dual Channels:** Supports two differential inputs (Channel A and B).
*   **Programmable Gain:** 32, 64, or 128.
*   **Interface:** 2-wire (Clock and Data) serial interface.

## 🔌 Wiring Diagram

| HX711 Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VCC | 3.3V or 5V | Red | |
| GND | GND | Black | |
| DT (Data) | GPIO 5 (Pin 29) | Blue | Serial Data Output |
| SCK (Clock)| GPIO 6 (Pin 31) | Yellow | Serial Clock Input |

### Load Cell to HX711 (Typical 4-wire)
| Load Cell Wire | HX711 Pin | Note |
| :--- | :--- | :--- |
| Red | E+ | Excitation + |
| Black | E- | Excitation - |
| White | A- | Signal - |
| Green | A+ | Signal + |

---

## 🚀 Quick Start

1.  **Run the Node:**
    ```bash
    ros2 launch xpi_sensors hx711.launch.py
    ```
2.  **Tare (Zero out):**
    ```bash
    ros2 service call /hx711/tare std_srvs/srv/Trigger {}
    ```
3.  **Monitor Weight:**
    ```bash
    ros2 topic echo /hx711/weight
    ```

## ⚖️ Calibration
To get weight in grams/kg, you must find your **Reference Unit**:
1. Run the node and observe raw values.
2. Place a known weight (e.g., 500g) on the scale.
3. Calculate: `reference_unit = (Measured Raw Value - Tare Value) / Known Weight`.
4. Pass this value as a parameter to the node.

## 📊 Published Topics
*   `~/weight` ([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html)): Calculated weight based on reference unit.
*   `~/raw` ([std_msgs/Int32](http://docs.ros.org/en/api/std_msgs/html/msg/Int32.html)): Raw 24-bit ADC value.
