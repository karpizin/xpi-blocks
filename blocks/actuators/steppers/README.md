# Unipolar Stepper Motor Driver (ULN2003)

This block provides a ROS2 driver for controlling a unipolar stepper motor (e.g., 28BYJ-48) via a ULN2003 driver board. It supports moving a specified number of steps or continuous rotation at a given speed.

## 📦 Bill of Materials
*   Raspberry Pi
*   Unipolar Stepper Motor (e.g., 28BYJ-48)
*   ULN2003 Stepper Driver Board
*   Jumper Wires
*   5V Power Supply (often from Pi, but for multiple motors/higher current, consider external)

## 🔌 Wiring
Connect the ULN2003 driver board to the Raspberry Pi's GPIO pins. The motor itself connects to the ULN2003 board.

| ULN2003 Pin | Raspberry Pi GPIO (BCM) | Note                                      |
|-------------|-------------------------|-------------------------------------------|
| IN1         | GPIO 17                 | Configurable via `in1_pin` parameter      |
| IN2         | GPIO 27                 | Configurable via `in2_pin` parameter      |
| IN3         | GPIO 22                 | Configurable via `in3_pin` parameter      |
| IN4         | GPIO 23                 | Configurable via `in4_pin` parameter      |
| VCC         | 5V                      | Power for ULN2003 and motor (often from Pi) |
| GND         | GND                     | Common Ground                             |

## 🚀 Quick Start
1.  **Ensure GPIO access:** Your user needs to be in the `gpio` group.
2.  **Launch the stepper motor driver:**
    ```bash
    ros2 launch xpi_actuators unipolar_stepper.launch.py
    ```

## 📡 Interface
### Subscribers
*   `~/cmd_steps` (`std_msgs/Int32`): Move the motor by the specified number of steps. Positive for clockwise, negative for counter-clockwise.
*   `~/cmd_speed` (`std_msgs/Float32`): Set the motor speed for continuous rotation.
    *   Value: `-1.0` (full speed reverse) to `1.0` (full speed forward), `0.0` (stop).
    *   Speed is relative to `step_delay_ms`.

### Parameters
*   `in1_pin` (int, default: `17`): BCM GPIO for ULN2003 IN1.
*   `in2_pin` (int, default: `27`): BCM GPIO for ULN2003 IN2.
*   `in3_pin` (int, default: `22`): BCM GPIO for ULN2003 IN3.
*   `in4_pin` (int, default: `23`): BCM GPIO for ULN2003 IN4.
*   `steps_per_revolution` (int, default: `2048`): Number of steps for a full revolution (common for 28BYJ-48 with gear reduction).
*   `step_delay_ms` (int, default: `2`): Delay in milliseconds between each step. Lower values mean faster rotation.
*   `mock_hardware` (bool, default: `false`): Run in mock mode.

## ✅ Verification
1.  Launch the driver with your stepper motor and ULN2003 connected.
2.  Send commands:
    *   Move 1024 steps clockwise (half revolution for 28BYJ-48):
        ```bash
        ros2 topic pub --once /unipolar_stepper_driver/cmd_steps std_msgs/msg/Int32 "{data: 1024}"
        ```
    *   Rotate continuously forward at half speed:
        ```bash
        ros2 topic pub --once /unipolar_stepper_driver/cmd_speed std_msgs/msg/Float32 "{data: 0.5}"
        ```
    *   Stop continuous rotation:
        ```bash
        ros2 topic pub --once /unipolar_stepper_driver/cmd_speed std_msgs/msg/Float32 "{data: 0.0}"
        ```
3.  Observe the stepper motor moving.

## ⚠️ Troubleshooting
*   **Motor not moving / vibrating?**
    *   Double-check wiring of IN1-IN4 to GPIOs.
    *   Ensure motor is powered (5V).
    *   Check `step_delay_ms`. If it's too low, the motor might not keep up.
    *   Verify `steps_per_revolution` is correct for your motor.
*   **Permissions error?**
    *   Add your user to the `gpio` group.
