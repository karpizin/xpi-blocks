# Direct GPIO Servo

This block allows you to control a single servo motor directly from a Raspberry Pi GPIO pin, without needing an external driver board (like PCA9685).

It uses software PWM (via `gpiozero`), which is perfect for non-critical applications like moving a camera mount, a latch, or a flag.

## 📦 Bill of Materials
*   Raspberry Pi
*   Servo Motor (SG90, MG996R, etc.)
*   Jumper Wires

## 🔌 Wiring
Connect the Servo directly to the Pi headers:

*   **VCC (Red)** <-> **5V** (Pin 2 or 4) *Note: Ensure your servo can handle 5V and doesn't draw too much current for the Pi's rail.*
*   **GND (Brown/Black)** <-> **GND** (Pin 6, 9, 14, etc.)
*   **Signal (Orange/Yellow)** <-> **Any GPIO Pin** (e.g., GPIO 18)

*Warning: For large servos (like MG996R) or under heavy load, POWER THE SERVO EXTERNALLY. The Pi's 5V rail has limited current.*

## 🚀 Quick Start
1.  **Build the workspace:**
    ```bash
    colcon build --packages-select xpi_actuators
    source install/setup.bash
    ```
2.  **Launch the driver:**
    ```bash
    ros2 launch xpi_actuators direct_servo.launch.py pin:=18
    ```

3.  **Move the servo:**
    ```bash
    # Center
    ros2 topic pub --once /direct_servo/cmd std_msgs/msg/Float32 "data: 0.0"
    
    # Max Left/CCW
    ros2 topic pub --once /direct_servo/cmd std_msgs/msg/Float32 "data: -1.0"
    
    # Max Right/CW
    ros2 topic pub --once /direct_servo/cmd std_msgs/msg/Float32 "data: 1.0"
    ```

## 📡 Interface
### Topics
| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `~/cmd` | `std_msgs/Float32` | Input | Target position. Range: `[-1.0, 1.0]`. |

### Parameters
*   `pin` (int, default: `18`): GPIO pin connected to signal.
*   `min_pulse` (float, default: `0.001`): Min pulse width (1ms).
*   `max_pulse` (float, default: `0.002`): Max pulse width (2ms).
*   `initial_value` (float, default: `0.0`): Startup position.

## ⚠️ Troubleshooting
*   **Jitter:** The servo twitches or shakes.
    *   *Cause:* Software PWM on Linux is not real-time. System load affects timing.
    *   *Fix:* Use the `pigpio` pin factory for better stability (requires `pigpiod` daemon running), or switch to the [PCA9685 Hardware Driver](../README.md).
