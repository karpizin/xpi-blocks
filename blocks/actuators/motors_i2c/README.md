# I2C DC Motor Driver (PCA9685 + TB6612)

This block provides a ROS2 driver for DC motor HATs that use an I2C **PCA9685** PWM expander to control a **TB6612FNG** (or L298) motor driver. This is a very common configuration for Raspberry Pi robot kits (e.g., Waveshare Motor Driver HAT).

## 🧠 Theory of Operation
The PCA9685 chip generates PWM signals via I2C. These signals are connected to the TB6612FNG driver inputs:
*   **PWM Pin:** Controls the motor speed.
*   **IN1/IN2 Pins:** Control the motor direction.

The node translates speed commands (-1.0 to 1.0) into the appropriate PWM duty cycles and logic levels on the PCA9685 channels.

## 📦 Bill of Materials
*   Raspberry Pi (4, 5, or Zero)
*   Waveshare Motor Driver HAT (or similar PCA9685-based motor driver)
*   Two DC Motors

## 🔌 Wiring
Usually, these modules are HATs that plug directly onto the Raspberry Pi GPIO header. If using a custom board:

| PCA9685 Pin | Raspberry Pi | Note |
|-------------|--------------|---------------------------|
| VCC         | 3.3V         | |
| SDA         | SDA (GPIO 2) | |
| SCL         | SCL (GPIO 3) | |
| GND         | GND          | |

## 🚀 Usage

**Launch the node:**
```bash
ros2 run xpi_actuators pca9685_motor_node
```

**Control Motor A:**
```bash
ros2 topic pub /pca9685_motor_node/motor_a/cmd_speed std_msgs/msg/Float32 "{data: 0.5}" --once
```

## 📡 Interface

### Subscribers
*   `~/motor_a/cmd_speed` (`std_msgs/Float32`): Speed from -1.0 to 1.0.
*   `~/motor_b/cmd_speed` (`std_msgs/Float32`): Speed from -1.0 to 1.0.

### Parameters
*   `i2c_bus` (int, default: `1`)
*   `i2c_address` (int, default: `0x40`)
*   `pwm_frequency` (float, default: `1000.0`): PWM frequency for the motors.
*   **Channel Mapping:**
    *   `motor_a_pwm` (int, default: `0`)
    *   `motor_a_in1` (int, default: `1`)
    *   `motor_a_in2` (int, default: `2`)
    *   `motor_b_pwm` (int, default: `5`)
    *   `motor_b_in1` (int, default: `3`)
    *   `motor_b_in2` (int, default: `4`)
