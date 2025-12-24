# Range: VL53L1X Time-of-Flight (ToF) Sensor (I2C)

The VL53L1X is a state-of-the-art, Time-of-Flight (ToF), laser-ranging sensor, enhancing the ST FlightSense™ product family. It is the fastest miniature ToF sensor on the market with accurate ranging up to 4 m and fast ranging frequency up to 50 Hz.

## 📌 Features
*   **Long distance:** Up to 400 cm (4 meters).
*   **Field of View (FoV):** 27°.
*   **Immunity to target color:** Unlike IR proximity sensors, laser ToF measures distance based on the speed of light, not intensity.
*   **Interface:** I2C (Address 0x29).

## 🔌 Wiring Diagram

| VL53L1X Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VIN | 3.3V (Pin 1) | Red | |
| GND | GND (Pin 9) | Black | |
| SCL | SCL (GPIO 3 / Pin 5) | Yellow | |
| SDA | SDA (GPIO 2 / Pin 3) | Blue | |
| XSHUT | Optional GPIO | - | Pull LOW to shutdown/reset |

## 🚀 Quick Start

1.  **Enable I2C:** Use `raspi-config` to enable the I2C interface.
2.  **Install Dependencies:**
    ```bash
    pip install vl53l1x
    ```
3.  **Run the Node:**
    ```bash
    ros2 launch xpi_sensors vl53l1x.launch.py
    ```
4.  **Monitor Range:**
    ```bash
    ros2 topic echo /vl53l1x/range
    ```

## 📊 Published Topics
*   `~/range` ([sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)): Distance measurement in meters.
