# IMU: BNO055 Intelligent 9-DOF Orientation Sensor (I2C)

The BNO055 is a System in Package (SiP), integrating a triaxial 14-bit accelerometer, a triaxial 16-bit gyroscope with a range of ±2000 degrees per second, a triaxial geomagnetic sensor and a 32-bit ARM Cortex M0+ microcontroller running Bosch Sensortec sensor fusion software.

## 📌 Features
*   **Absolute Orientation:** Euler angles and Quaternions.
*   **Sensor Fusion:** All the math is done on-chip (no drift issues).
*   **Linear Acceleration:** Gravity is automatically filtered out.
*   **Interface:** I2C (Address 0x28 or 0x29).

## 🔌 Wiring Diagram

| BNO055 Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VIN | 3.3V (Pin 1) | Red | |
| GND | GND (Pin 9) | Black | |
| SCL | SCL (GPIO 3 / Pin 5) | Yellow | |
| SDA | SDA (GPIO 2 / Pin 3) | Blue | |

> **Note:** On Raspberry Pi, ensure the `i2c_arm_baudrate=100000` is set in `/boot/config.txt` for stable communication.

## 🚀 Quick Start

1.  **Enable I2C:** Use `raspi-config` to enable the I2C interface.
2.  **Install Dependencies:**
    ```bash
    pip install adafruit-circuitpython-bno055
    ```
3.  **Run the Node:**
    ```bash
    ros2 launch xpi_sensors bno055.launch.py
    ```
4.  **Monitor Orientation:**
    ```bash
    ros2 topic echo /bno055/imu
    ```

## 📊 Published Topics
*   `~/imu` ([sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)): Orientation, angular velocity, and linear acceleration.
*   `~/euler` ([geometry_msgs/Vector3](http://docs.ros.org/en/api/geometry_msgs/html/msg/Vector3.html)): Roll, Pitch, Yaw in degrees.
*   `~/status` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)): Calibration status.
