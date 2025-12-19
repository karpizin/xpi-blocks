# QMC5883L: 3-Axis Digital Compass

The **QMC5883L** is a surface-mount, multi-chip module designed for low-field magnetic sensing. It allows the robot to determine its orientation relative to the Earth's magnetic north.

## ⚡ Wiring (I2C)

| QMC5883L Pin | Raspberry Pi Pin | Description |
| :--- | :--- | :--- |
| **VCC** | 3.3V | Power. |
| **GND** | GND | Ground. |
| **SDA** | GPIO 2 (Pin 3) | I2C Data. |
| **SCL** | GPIO 3 (Pin 5) | I2C Clock. |

## 🚀 Usage

### 1. Launch the Node
```bash
ros2 launch xpi_sensors magnetometer.launch.py
```

### 2. Verify Data
```bash
ros2 topic echo /magnetometer/heading
```
*Output: Angle in degrees (0 = North, 90 = East).*

## ⚠️ Calibration (Mandatory)
Magnetometers are sensitive to metal parts of the robot (motors, batteries).
1.  Launch the node and rotate the robot 360 degrees in all axes.
2.  Note the min/max values for X, Y, and Z.
3.  Enter these as `offset` and `scale` parameters in the launch file.

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `i2c_bus` | int | `1` | I2C bus number. |
| `declination` | float | `0.0` | Magnetic declination for your city (in radians). |
| `polling_rate` | float | `10.0` | Hz. |

## 🧩 Topics Interface

### Publishers
*   `~/magnetic_field` (`sensor_msgs/msg/MagneticField`) - Raw calibrated data in Tesla.
*   `~/heading` (`std_msgs/msg/Float32`) - Heading in degrees (0-360).
