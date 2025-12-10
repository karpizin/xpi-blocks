# ESC Motor Driver

This block enables control of Brushless DC (BLDC) motors via standard Electronic Speed Controllers (ESC) using the ROS2 framework. It supports both unidirectional (drone/plane) and bidirectional (car/rover) ESCs.

## 📦 Bill of Materials
*   Raspberry Pi
*   ESC (Electronic Speed Controller) - Standard PWM interface.
*   BLDC Motor (compatible with your ESC).
*   Power Source (LiPo Battery usually required for motors).

## 🔌 Wiring
Connect the ESC signal wire (usually White or Yellow) to a GPIO pin (e.g., GPIO 12).
Connect the ESC Ground wire (Black or Brown) to a Raspberry Pi Ground pin.
**Important:** Do NOT connect the ESC's 5V (Red) wire to the Raspberry Pi if the ESC has a BEC (Battery Elimination Circuit) unless you are powering the Pi from it and know what you are doing. Usually, it's safer to disconnect the Red wire.

*   **Signal** <-> **GPIO 12** (PWM capable pin recommended for hardware timing, though software PWM works).
*   **GND** <-> **GND**

## 🚀 Quick Start
1.  **Build the workspace:**
    ```bash
    colcon build --packages-select xpi_actuators
    source install/setup.bash
    ```
2.  **Launch the driver (Unidirectional - Drone/Plane):**
    ```bash
    ros2 launch xpi_actuators esc_driver.launch.py pin:=12 stop_value:=-1.0
    ```
    *Note: `stop_value:=-1.0` sends min pulse (1ms) at rest.*

3.  **Launch the driver (Bidirectional - Car/Rover):**
    ```bash
    ros2 launch xpi_actuators esc_driver.launch.py pin:=12 stop_value:=0.0
    ```
    *Note: `stop_value:=0.0` sends neutral pulse (1.5ms) at rest.*

## 📡 Interface
### Topics
| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `~/cmd` | `std_msgs/Float32` | Input | Motor command. Range: `[-1.0, 1.0]`. |

### Parameters
*   `pin` (int, default: `12`): GPIO pin connected to ESC signal.
*   `min_pulse` (float, default: `0.001`): Min pulse width (1ms).
*   `max_pulse` (float, default: `0.002`): Max pulse width (2ms).
*   `stop_value` (float, default: `-1.0`): Value to send on safety timeout or startup.
    *   `-1.0`: For unidirectional ESCs (Stop).
    *   `0.0`: For bidirectional ESCs (Neutral).
*   `timeout` (float, default: `0.5`): Time in seconds before motor stops if no command is received.

## ✅ Verification
1.  **Run the node (Unidirectional example):**
    ```bash
    ros2 launch xpi_actuators esc_driver.launch.py
    ```
    *You should hear the ESC initialization beeps.*

2.  **Send a command (20% throttle):**
    ```bash
    ros2 topic pub --once /esc_driver/cmd std_msgs/msg/Float32 "data: -0.6"
    ```
    *Note: Range is -1 (Stop) to 1 (Max). So -0.6 is 20% throttle ([-1 ... 1] span is 2.0. -1 + 0.4 = -0.6).*
    
    *Easier Calculation:*
    *   Unidirectional: Maps `[-1, 1]` to `[1ms, 2ms]`.
    *   Stop: `-1.0`
    *   Half speed: `0.0`
    *   Full speed: `1.0`

3.  **Monitor Safety Stop:**
    Stop publishing. After 0.5s, the motor should stop (ESC receives `stop_value`).

## ⚠️ Troubleshooting
*   **Motor just beeps continuously:** The ESC is not "armed". It usually waits for a low signal (1ms) for a few seconds. Ensure `stop_value` is set correctly (`-1.0` for most) and the node is running.
*   **Motor jitters:** Software PWM might be unstable under load. Try to use hardware PWM pins (GPIO 12, 13, 18, 19) and ensure `pigpio` daemon is running if you want hardware timing (though this node uses default `gpiozero` factory).
