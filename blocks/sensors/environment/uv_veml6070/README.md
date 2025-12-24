# Weather Station: VEML6070 UV Light Sensor (I2C)

The VEML6070 is an advanced ultraviolet (UV) light sensor with an I2C interface. It is designed to sense the UV spectrum (UVA) and provide a digital output proportional to the UV intensity.

## 📌 Features
*   **Spectral Sensitivity:** 320nm to 410nm (UVA).
*   **Programmable Integration Time:** Allows for sensitivity adjustment.
*   **Interface:** I2C (uses two addresses: 0x38 for write, 0x39 for read).
*   **Output:** UV intensity in raw counts and UV Index estimation.

## 🔌 Wiring Diagram

| VEML6070 Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VCC | 3.3V (Pin 1) | Red | |
| GND | GND (Pin 9) | Black | |
| SCL | SCL (GPIO 3 / Pin 5) | Yellow | |
| SDA | SDA (GPIO 2 / Pin 3) | Blue | |
| ACK | - | - | Optional interrupt (not used) |

## 🚀 Quick Start

1.  **Enable I2C:** Use `raspi-config` to enable the I2C interface.
2.  **Run the Node:**
    ```bash
    ros2 launch xpi_sensors veml6070.launch.py
    ```
3.  **Monitor UV Intensity:**
    ```bash
    ros2 topic echo /veml6070/uv_raw
    ros2 topic echo /veml6070/uv_index_level
    ```

## 📊 Published Topics
*   `~/uv_raw` ([std_msgs/Int32](http://docs.ros.org/en/api/std_msgs/html/msg/Int32.html)): Raw UV intensity value.
*   `~/uv_index_level` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)): Risk level description (e.g., "Low", "Moderate", "Very High").
