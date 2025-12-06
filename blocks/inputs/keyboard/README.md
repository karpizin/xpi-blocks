# Keyboard Input (Teleoperation)

This block provides a simple way to control a robot using keyboard commands. It leverages the existing `teleop_twist_keyboard` ROS2 package.

## 📦 Bill of Materials
*   Raspberry Pi with connected monitor and keyboard (or SSH access with a terminal).

## 🔌 Setup
No specific wiring is needed, as this uses your system's keyboard input.

## 🚀 Quick Start
1.  **Install `teleop_twist_keyboard`**:
    ```bash
    sudo apt update
    sudo apt install ros-humble-teleop-twist-keyboard
    ```
2.  **Launch the keyboard teleop driver**:
    ```bash
    ros2 launch xpi_inputs keyboard.launch.py
    ```
    *   This will open a new terminal window (using `xterm`) where you can enter commands.
    *   **IMPORTANT:** The launched `xterm` window *must* have focus for key presses to register.

## 📡 Interface
### Publishers
*   `/keyboard_teleop/cmd_vel` (`geometry_msgs/Twist`): Publishes linear and angular velocity commands based on key presses.

### Parameters (Configurable in launch file)
*   `speed` (float, default: `0.5`): Linear speed in meters/second.
*   `turn` (float, default: `1.0`): Angular speed in radians/second.

## ✅ Verification
1.  Launch the driver as described above.
2.  In a separate terminal, monitor the `/keyboard_teleop/cmd_vel` topic:
    ```bash
    ros2 topic echo /keyboard_teleop/cmd_vel
    ```
3.  Switch focus to the `xterm` window launched by `keyboard.launch.py` and press keys (e.g., `i` for forward, `j`/`l` for turn). You should see `Twist` messages being published.

## ⚠️ Troubleshooting
*   **No `xterm`?** `xterm` needs to be installed: `sudo apt install xterm`.
*   **Key presses not registering?** Ensure the `xterm` window has keyboard focus.
*   **"teleop_twist_keyboard not found"?** Make sure the package is installed as per step 1.
