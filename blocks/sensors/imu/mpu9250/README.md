# IMU: MPU9250 9-DOF Orientation Sensor (I2C)

The MPU-9250 is a multi-chip module (MCM) consisting of two dies integrated into a single QFN package. One die houses the 3-Axis gyroscope and the 3-Axis accelerometer. The other die houses the AK8963 3-Axis magnetometer.

## 📌 Features
*   **Accelerometer:** 3-Axis, up to ±16g.
*   **Gyroscope:** 3-Axis, up to ±2000 dps.
*   **Magnetometer:** 3-Axis AK8963, up to ±4800 μT.
*   **Interface:** I2C (Address 0x68).

## 🔌 Wiring Diagram

| MPU9250 Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VCC | 3.3V (Pin 1) | Red | |
| GND | GND (Pin 9) | Black | |
| SCL | SCL (GPIO 3 / Pin 5) | Yellow | |
| SDA | SDA (GPIO 2 / Pin 3) | Blue | |
| AD0 | - | - | Connect to GND for 0x68, VCC for 0x69 |

---

## 🚀 Quick Start

1.  **Enable I2C:** Use `raspi-config`.
2.  **Install Dependencies:**
    ```bash
    pip install mpu9250-jmdev
    ```
3.  **Run the Node:**
    ```bash
    ros2 launch xpi_sensors mpu9250.launch.py
    ```
4.  **Monitor IMU Data:**
    ```bash
    ros2 topic echo /mpu9250/imu
    ```

## 📊 Published Topics
*   `~/imu` ([sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)): Acceleration and angular velocity.
*   `~/mag` ([sensor_msgs/MagneticField](http://docs.ros.org/en/api/sensor_msgs/html/msg/MagneticField.html)): 3-Axis magnetic field data.
*   `~/temp` ([sensor_msgs/Temperature](http://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html)): Internal sensor temperature.
