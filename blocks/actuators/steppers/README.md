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

---

# A4988 / DRV8825 Bipolar Stepper Motor Driver (Step/Dir)

This block provides a ROS2 driver for controlling bipolar stepper motors using common Step/Dir drivers like A4988 or DRV8825. These drivers offer microstepping capabilities for smoother and quieter operation.

## 📦 Bill of Materials
*   Raspberry Pi
*   Bipolar Stepper Motor (e.g., NEMA17)
*   A4988 or DRV8825 Stepper Driver Module
*   External Power Supply (e.g., 8V-35V for A4988, up to 45V for DRV8825) for the motor. **Do NOT power the motor from the Pi!**
*   Jumper Wires

## 🔌 Wiring
Connect the A4988/DRV8825 driver board to the Raspberry Pi's GPIO pins. The stepper motor connects to the driver board.

| A4988/DRV8825 Pin | Raspberry Pi GPIO (BCM) | Note                                      |
|-------------------|-------------------------|-------------------------------------------|
| STEP              | GPIO 17                 | Configurable via `step_pin` parameter     |
| DIR               | GPIO 27                 | Configurable via `dir_pin` parameter      |
| ENABLE            | GPIO 22                 | Optional, configurable via `enable_pin` (active low) |
| MS1, MS2, MS3     | GPIO 5, 6, 13           | Configurable via `microstep_pins` (for microstepping) |
| VMOT              | External PSU +          | Motor Power Supply                        |
| GND               | External PSU - & RPi GND| Common Ground                             |
| VDD               | 3.3V/5V                 | Logic Power for Driver (often 3.3V from Pi) |

**Important Note on Microstepping Pins:**
A4988 typically uses MS1, MS2, MS3 for 1, 1/2, 1/4, 1/8, 1/16 microstepping. DRV8825 uses MS0, MS1, MS2 for up to 1/32 microstepping. Refer to your driver's datasheet for exact pin functions and truth table. This driver assumes typical connections.

## 🚀 Quick Start
1.  **Ensure GPIO access:** Your user needs to be in the `gpio` group.
2.  **Adjust current limit:** Most A4988/DRV8825 boards have a small potentiometer. Adjust it to set the current limit according to your motor's specifications.
3.  **Launch the stepper motor driver:**
    ```bash
    # Example: A4988 with 1/16 microstepping, enable pin on GPIO 22, MS pins on 5,6,13
    ros2 launch xpi_actuators a4988.launch.py \
        step_pin:=17 dir_pin:=27 enable_pin:=22 \
        microstep_pins:="[5, 6, 13]" microstep_setting:=16
    ```
    *Note: The `microstep_pins` argument must be passed as a JSON-formatted string representing a list.*

## 📡 Interface
### Subscribers
*   `~/cmd_steps` (`std_msgs/Int32`): Move the motor by the specified number of steps. Positive for clockwise, negative for counter-clockwise.
    *   This refers to *microsteps*.
*   `~/cmd_speed` (`std_msgs/Float32`): Set the motor speed for continuous rotation.
    *   Value: `-1.0` (full speed reverse) to `1.0` (full speed forward), `0.0` (stop).
    *   Speed is relative to `motor_max_rpm`.

### Parameters
*   `step_pin` (int, default: `17`): BCM GPIO pin for STEP signal.
*   `dir_pin` (int, default: `27`): BCM GPIO pin for DIR signal.
*   `enable_pin` (int, default: `None`): Optional BCM GPIO pin for ENABLE (active low). Set to `None` if not used.
*   `microstep_pins` (list of int, default: `[]`): List of BCM GPIO pins for MS1, MS2, MS3 (or MS0, MS1, MS2 for DRV8825).
*   `microstep_setting` (int, default: `16`): Microstep resolution (1, 2, 4, 8, 16, etc.).
*   `steps_per_revolution_fullstep` (int, default: `200`): Number of full steps per motor revolution (e.g., 200 for 1.8 degree).
*   `step_delay_s` (float, default: `0.001`): Minimum delay in seconds between each step pulse.
*   `motor_max_rpm` (int, default: `60`): Maximum RPM to calculate continuous speed.
*   `mock_hardware` (bool, default: `false`): Run in mock mode.

## ✅ Verification
1.  Launch the driver with your stepper motor and driver board connected.
2.  Send commands:
    *   Move 200 microsteps clockwise (e.g., 1/16 microstep on a 1.8 deg motor means 1/16 of a full step):
        ```bash
        ros2 topic pub --once /a4988_stepper_driver/cmd_steps std_msgs/msg/Int32 "{data: 200}"
        ```
    *   Rotate continuously counter-clockwise at half speed:
        ```bash
        ros2 topic pub --once /a4988_stepper_driver/cmd_speed std_msgs/msg/Float32 "{data: -0.5}"
        ```
    *   Stop continuous rotation:
        ```bash
        ros2 topic pub --once /a4988_stepper_driver/cmd_speed std_msgs/msg/Float32 "{data: 0.0}"
        ```
3.  Observe the stepper motor moving.

## ⚠️ Troubleshooting
*   **Motor not moving / vibrating?**
    *   Double-check wiring of STEP, DIR, ENABLE, MS pins to GPIOs.
    *   Ensure external motor power supply is connected and providing adequate voltage/current.
    *   Adjust the current limit potentiometer on the driver board.
    *   Check `step_delay_s`. If it's too low, the motor might not keep up.
    *   Verify `steps_per_revolution_fullstep` and `microstep_setting` are correct.
    *   Check for permissions errors (`sudo usermod -a -G gpio $USER`).
*   **Motor moving in wrong direction?**
    *   Toggle the `DIR` pin logic (swap physical connection or invert in software if driver allows).

---

# L298 / L293 Stepper and Dual DC Motor Driver

This block provides a ROS2 driver for controlling bipolar stepper motors or two DC motors using the versatile L298 or L293 motor driver boards. These drivers are capable of handling higher currents and voltages compared to some smaller alternatives, making them suitable for more powerful motors.

## 📦 Bill of Materials
*   Raspberry Pi
*   L298N or L293D Motor Driver Module
*   Bipolar Stepper Motor OR 2x DC Motors
*   External Power Supply (e.g., 5V-35V) for motors. **Do NOT power motors from the Pi!**
*   Jumper Wires

## 🔌 Wiring
Connect the L298/L293 driver board to the Raspberry Pi's GPIO pins. The motors connect to the driver board.

### Stepper Motor Mode Wiring Example (Full-step, 4-wire bipolar)

| L298/L293 Pin | Raspberry Pi GPIO (BCM) | Note                                      |
|---------------|-------------------------|-------------------------------------------|
| IN1           | GPIO 17                 | Configurable via `stepper_in1_pin`        |
| IN2           | GPIO 27                 | Configurable via `stepper_in2_pin`        |
| IN3           | GPIO 22                 | Configurable via `stepper_in3_pin`        |
| IN4           | GPIO 23                 | Configurable via `stepper_in4_pin`        |
| ENA (optional)| GPIO 5                  | Optional, configurable via `stepper_enable_pin` (active high) |
| +V_MOTOR      | External PSU +          | Motor Power Supply                        |
| GND           | External PSU - & RPi GND| Common Ground                             |
| +5V (optional)| 5V                      | If L298 has a 5V regulator, can power Pi logic |

### Dual DC Motor Mode Wiring Example

| L298/L293 Pin | Raspberry Pi GPIO (BCM) | Note                                      |
|---------------|-------------------------|-------------------------------------------|
| **Motor A**   |                         |                                           |
| IN1           | GPIO 17                 | Configurable via `motor_a_in1_pin`        |
| IN2           | GPIO 27                 | Configurable via `motor_a_in2_pin`        |
| ENA           | GPIO 5 (PWM)            | Optional, configurable via `motor_a_pwm_pin` |
|               |                         |                                           |
| **Motor B**   |                         |                                           |
| IN3           | GPIO 22                 | Configurable via `motor_b_in1_pin`        |
| IN4           | GPIO 23                 | Configurable via `motor_b_in2_pin`        |
| ENB           | GPIO 6 (PWM)            | Optional, configurable via `motor_b_pwm_pin` |
|               |                         |                                           |
| **Common**    |                         |                                           |
| +V_MOTOR      | External PSU +          | Motor Power Supply                        |
| GND           | External PSU - & RPi GND| Common Ground                             |
| +5V (optional)| 5V                      | If L298 has a 5V regulator, can power Pi logic |

## 🚀 Quick Start
1.  **Ensure GPIO access:** Your user needs to be in the `gpio` group.
2.  **Launch the L298 driver in Stepper Mode:**
    ```bash
    ros2 launch xpi_actuators l298.launch.py driver_mode:=stepper \
        stepper_in1_pin:=17 stepper_in2_pin:=27 stepper_in3_pin:=22 stepper_in4_pin:=23
    ```
3.  **Launch the L298 driver in Dual DC Motor Mode:**
    ```bash
    ros2 launch xpi_actuators l298.launch.py driver_mode:=dc_dual \
        motor_a_in1_pin:=17 motor_a_in2_pin:=27 motor_a_pwm_pin:=5 \
        motor_b_in1_pin:=22 motor_b_in2_pin:=23 motor_b_pwm_pin:=6
    ```

## 📡 Interface
The interface varies based on `driver_mode` parameter.

### Stepper Mode Subscribers
*   `~/cmd_steps` (`std_msgs/Int32`): Move the motor by the specified number of steps. Positive for clockwise, negative for counter-clockwise.
*   `~/cmd_speed` (`std_msgs/Float32`): Set the motor speed for continuous rotation.
    *   Value: `-1.0` (full speed reverse) to `1.0` (full speed forward), `0.0` (stop).
    *   Speed is relative to `stepper_motor_max_rpm`.

### Dual DC Motor Mode Subscribers
*   `~/motor_a/cmd_speed` (`std_msgs/Float32`): Speed command for Motor A.
    *   Value: `-1.0` (full reverse) to `1.0` (full forward), `0.0` (stop).
*   `~/motor_b/cmd_speed` (`std_msgs/Float32`): Speed command for Motor B.
    *   Value: `-1.0` (full reverse) to `1.0` (full forward), `0.0` (stop).

### Parameters
*   `driver_mode` (string, default: `stepper`): Choose 'stepper' or 'dc_dual'.
*   `mock_hardware` (bool, default: `false`): Run in mock mode.

*   **Stepper Mode Specific Parameters:**
    *   `stepper_in1_pin`, `stepper_in2_pin`, `stepper_in3_pin`, `stepper_in4_pin` (int, defaults: `17, 27, 22, 23`): BCM GPIO pins for IN1-IN4.
    *   `stepper_enable_pin` (int, default: `None`): Optional BCM GPIO pin for ENABLE (active high).
    *   `stepper_steps_per_revolution` (int, default: `200`): Full steps for a revolution.
    *   `stepper_step_delay_s` (float, default: `0.005`): Min delay between steps.
    *   `stepper_motor_max_rpm` (int, default: `60`): Max RPM for speed calculation.

*   **Dual DC Motor Mode Specific Parameters:**
    *   `motor_a_in1_pin`, `motor_a_in2_pin` (int, defaults: `17, 27`): BCM GPIO pins for Motor A direction.
    *   `motor_a_pwm_pin` (int, default: `None`): Optional BCM GPIO pin for Motor A PWM (ENA).
    *   `motor_b_in1_pin`, `motor_b_in2_pin` (int, defaults: `22, 23`): BCM GPIO pins for Motor B direction.
    *   `motor_b_pwm_pin` (int, default: `None`): Optional BCM GPIO pin for Motor B PWM (ENB).
    *   `dc_pwm_frequency` (int, default: `1000`): PWM frequency for DC motor speed.

## ✅ Verification
### Stepper Mode
1.  Launch in stepper mode.
2.  Send commands to `~/cmd_steps` or `~/cmd_speed`.
    ```bash
    ros2 topic pub --once /l298_motor_driver/cmd_steps std_msgs/msg/Int32 "{data: 200}"
    ```
### Dual DC Motor Mode
1.  Launch in dual DC motor mode.
2.  Send commands to `~/motor_a/cmd_speed` or `~/motor_b/cmd_speed`.
    ```bash
    ros2 topic pub --once /l298_motor_driver/motor_a/cmd_speed std_msgs/msg/Float32 "{data: 0.8}"
    ```

## ⚠️ Troubleshooting
*   **Motors not moving / erratic behavior?**
    *   Double-check all wiring (IN1-IN4, ENA/ENB, V_MOTOR, GND).
    *   Ensure external power supply for motors is connected and appropriate for motor voltage.
    *   Verify GPIO pin numbers.
    *   Check `stepper_step_delay_s` for steppers.
    *   Check for permissions errors (`sudo usermod -a -G gpio $USER`).
*   **Motor moving in wrong direction?**
    *   Swap IN1/IN2 (for DC) or IN1/IN4 (for stepper).
*   **"Error initializing GPIO"**: Check `mock_hardware` parameter for testing without physical driver.


