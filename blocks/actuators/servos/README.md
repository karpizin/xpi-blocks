# PCA9685 PWM / Servo Driver

This block controls the PCA9685 16-channel 12-bit PWM driver. It is commonly used to control multiple servos or LEDs.

## 📦 Bill of Materials
*   Raspberry Pi
*   PCA9685 Module (I2C)
*   Servos (SG90, MG996R, etc.)
*   **External Power Supply** (5V/6V) for servos. **Do not power servos from the Pi!**

## 🔌 Wiring

| PCA9685 | Raspberry Pi | Note |
|Data | GPIO 2 (SDA) | Pin 3 |
| Clock | GPIO 3 (SCL) | Pin 5 |
| VCC | 3.3V (Pin 1) | Logic Power |
| GND | GND (Pin 6) | Common Ground |
| **V+ (Terminal)** | **External PSU +** | Power for Motors |
| **GND (Terminal)**| **External PSU -** | Power for Motors |

## 🚀 Quick Start
1. Enable I2C on Raspberry Pi (`sudo raspi-config` -> Interface Options -> I2C).
2. Install `i2c-tools` and check address:
   ```bash
   sudo apt install i2c-tools
   i2cdetect -y 1
   # You should see 40 (default) and 70 (All Call)
   ```
3. Run the node:
   ```bash
   ros2 launch xpi_actuators pca9685.launch.py
   ```

## 📡 Interface
### Subscribers
*   `~/cmd` (`std_msgs/Float32MultiArray`):
    *   Send array of floats [0.0 ... 1.0] representing duty cycle.
    *   Index 0 = Channel 0, Index 15 = Channel 15.
    *   **Servo Control:** Standard servos expect 50Hz.
        *   0.05 (5%) ≈ 1ms pulse (-90°)
        *   0.075 (7.5%) ≈ 1.5ms pulse (0°)
        *   0.10 (10%) ≈ 2ms pulse (+90°)
        *   *Note:* These values vary by servo model!

## ✅ Verification
Move Channel 0 Servo to center position (approx):

```bash
ros2 topic pub --once /pwm_driver/cmd std_msgs/msg/Float32MultiArray "{data: [0.075]}"
```
