# MPU6050 IMU (Inertial Measurement Unit) 6-DOF

This block provides a ROS2 driver for the MPU6050, a 6-axis (accelerometer + gyroscope) Inertial Measurement Unit. It reads raw acceleration, angular velocity, and temperature data via I2C and publishes them as `sensor_msgs/Imu` and `sensor_msgs/Temperature` messages.

## 📦 Bill of Materials
*   Raspberry Pi
*   MPU6050 module (often comes with a voltage regulator onboard)
*   Jumper Wires

## 🔌 Wiring
Connect the MPU6050 to the I2C pins on the Raspberry Pi.

| MPU6050 Pin | Raspberry Pi | Note |
|-------------|--------------|-----------------------------------|
| VCC         | 3.3V         | Power for the module (Pin 1)      |
| GND         | GND          | Common Ground (Pin 6)             |
| SDA         | GPIO 2 (SDA) | Data line (Pin 3)                 |
| SCL         | GPIO 3 (SCL) | Clock line (Pin 5)                |
| AD0         | GND (or 3.3V)| Sets I2C address (0x68 if GND, 0x69 if 3.3V) |

**Important I2C Configuration on Raspberry Pi:**
You must enable the I2C interface on your Raspberry Pi.

1.  **Enable I2C:**
    ```bash
    sudo raspi-config
    # Interface Options -> P3 I2C -> Yes
    ```
2.  **Verify I2C Address:** Check the MPU6050's I2C address (usually 0x68 or 0x69)
    ```bash
    sudo apt install i2c-tools
    i2cdetect -y 1 # Or 0 if using an older Pi
    # Look for '68' or '69' in the output.
    ```

## 🚀 Quick Start
1.  **Perform I2C Configuration** as described above.
2.  **Launch the MPU6050 driver**:
    ```bash
    ros2 launch xpi_sensors mpu6050.launch.py i2c_address:=0x68
    ```
    (Adjust `i2c_address` if yours is 0x69, or other parameters as needed).

## 📡 Interface
### Publishers
*   `~/imu/data_raw` (`sensor_msgs/Imu`): Publishes raw acceleration (m/s^2), angular velocity (rad/s). Orientation covariance is set to -1 (unknown) as no fusion is performed here.
*   `~/temperature` (`sensor_msgs/Temperature`): Publishes internal temperature in Celsius.

### Parameters
*   `i2c_bus` (int, default: `1`): I2C bus number (usually 1 for RPi).
*   `i2c_address` (int, default: `0x68`): I2C address of the MPU6050.
*   `publish_rate` (float, default: `50.0`): Frequency to publish data in Hz.
*   `accel_fsr` (int, default: `0`): Accelerometer Full Scale Range (0: +/-2g, 1: +/-4g, 2: +/-8g, 3: +/-16g).
*   `gyro_fsr` (int, default: `0`): Gyroscope Full Scale Range (0: +/-250dps, 1: +/-500dps, 2: +/-1000dps, 3: +/-2000dps).
*   `frame_id` (string, default: `imu_link`): Frame ID for messages.
*   `mock_hardware` (bool, default: `false`): Run in mock mode without real MPU6050 hardware.

## ✅ Verification
1.  Launch the driver with your MPU6050 connected and I2C enabled.
2.  In new terminals, monitor the published topics:
    ```bash
    ros2 topic echo /mpu6050_sensor/imu/data_raw
    ros2 topic echo /mpu6050_sensor/temperature
    ```
    You should see streams of IMU and temperature data.

## ⚠️ Troubleshooting
*   **"Failed to initialize MPU6050" / "I/O Error"**:
    *   Double-check I2C wiring (SDA/SCL, VCC/GND).
    *   Verify I2C is enabled (`sudo raspi-config`).
    *   Ensure the correct `i2c_address` is used.
    *   Check for I2C permissions (`sudo usermod -a -G i2c $USER`).
*   **No data / erratic data**:
    *   Check physical connections.
    *   Verify module is powered.
    *   Try different `publish_rate` values.
    *   Ensure proper grounding.
