# TB6612FNG Dual DC Motor Driver

This block provides a ROS2 driver for controlling two DC motors using the TB6612FNG motor driver module. It controls motor direction via GPIO pins and speed via PWM.

## 📦 Bill of Materials
*   Raspberry Pi
*   TB6612FNG Motor Driver Module
*   2x DC Motors
*   External Power Supply (e.g., 5V-12V) for motors. **Do not power motors from the Pi!**
*   Jumper Wires

## 🔌 Wiring
The TB6612FNG controls two motors, Motor A and Motor B. Each motor requires two IN pins for direction and one PWM pin for speed.

| TB6612FNG Pin | Raspberry Pi GPIO (BCM) | Note                                      |
|---------------|-------------------------|-------------------------------------------|
| **Motor A**   |                         |                                           |
| PWMA          | GPIO 12                 | Configurable via `motor_a_pwm_pin`        |
| AIN1          | GPIO 5                  | Configurable via `motor_a_in1_pin`        |
| AIN2          | GPIO 6                  | Configurable via `motor_a_in2_pin`        |
|               |                         |                                           |
| **Motor B**   |                         |                                           |
| PWMB          | GPIO 13                 | Configurable via `motor_b_pwm_pin`        |
| BIN1          | GPIO 26                 | Configurable via `motor_b_in1_pin`        |
| BIN2          | GPIO 19                 | Configurable via `motor_b_in2_pin`        |
|               |                         |                                           |
| **Common**    |                         |                                           |
| STBY          | (Connect to 3.3V)       | Keep high for normal operation.           |
| VM            | External PSU +          | Motor Power Supply                        |
| GND           | External PSU - & RPi GND| Common Ground (essential!)                |

## 🚀 Quick Start
1.  **Ensure GPIO access:** Your user needs to be in the `gpio` group.
2.  **Launch the motor driver:**
    ```bash
    ros2 launch xpi_actuators tb6612.launch.py
    ```

## 📡 Interface
### Subscribers
*   `~/motor_a/cmd_vel` (`std_msgs/Float32`): Speed command for Motor A.
    *   Value: `-1.0` (full reverse) to `1.0` (full forward), `0.0` (stop).
*   `~/motor_b/cmd_vel` (`std_msgs/Float32`): Speed command for Motor B.
    *   Value: `-1.0` (full reverse) to `1.0` (full forward), `0.0` (stop).

### Parameters
*   `motor_a_pwm_pin` (int, default: `12`): BCM GPIO pin for Motor A PWM.
*   `motor_a_in1_pin` (int, default: `5`): BCM GPIO pin for Motor A IN1.
*   `motor_a_in2_pin` (int, default: `6`): BCM GPIO pin for Motor A IN2.
*   `motor_b_pwm_pin` (int, default: `13`): BCM GPIO pin for Motor B PWM.
*   `motor_b_in1_pin` (int, default: `26`): BCM GPIO pin for Motor B IN1.
*   `motor_b_in2_pin` (int, default: `19`): BCM GPIO pin for Motor B IN2.
*   `pwm_frequency` (int, default: `1000`): PWM frequency in Hz.
*   `mock_hardware` (bool, default: `false`): Run in mock mode without real GPIO.

### Safety: command watchdog and speed limit
The driver stops a motor if no command arrives on its topic for `cmd_timeout` seconds
(dead-man switch). Publishers must **stream** commands (e.g. 10 Hz), not send a single
message. Non-finite values (`NaN`, `inf`) are treated as stop; values are clamped to `max_speed`.

*   `cmd_timeout` (float, default: `0.5`): seconds without a command before the motor is stopped. `0` disables the watchdog (not recommended).
*   `max_speed` (float, default: `1.0`): absolute limit applied to every command (0..1).

## ✅ Verification
1.  Launch the driver as described above.
2.  Send commands to Motor A:
    *   Forward at half speed (`-r 10` streams at 10 Hz; with `--once` the watchdog stops the motor after 0.5 s; press Ctrl+C to stop):
        ```bash
        ros2 topic pub -r 10 /motor_driver/motor_a/cmd_vel std_msgs/msg/Float32 "{data: 0.5}"
        ```
    *   Stop:
        ```bash
        ros2 topic pub --once /motor_driver/motor_a/cmd_vel std_msgs/msg/Float32 "{data: 0.0}"
        ```
    *   Reverse at full speed:
        ```bash
        ros2 topic pub -r 10 /motor_driver/motor_a/cmd_vel std_msgs/msg/Float32 "{data: -1.0}"
        ```
3.  Repeat for Motor B using `/motor_driver/motor_b/cmd_vel`.

## ⚠️ Troubleshooting
*   **Motors not moving?**
    *   Check all wiring carefully.
    *   Ensure external power supply for motors is connected and turned on.
    *   Verify GPIO pin numbers are correct.
    *   Check for permissions errors (`sudo usermod -a -G gpio $USER`).
*   **Motors move in wrong direction?**
    *   Swap `IN1` and `IN2` pin connections for that motor, or reverse the speed sign.
