# Joystick / Gamepad Input

This block integrates a standard USB/Bluetooth joystick or gamepad with ROS2, publishing its button and axis states to the `/joy` topic.

## 📦 Bill of Materials
*   Raspberry Pi
*   USB Gamepad (e.g., Xbox 360/One, PS4 controller) or Bluetooth Gamepad.

## 🔌 Wiring
Connect your USB gamepad directly to the Raspberry Pi.
For Bluetooth gamepads, pair them via `bluetoothctl` or your OS's Bluetooth settings.

## 🚀 Quick Start
1.  Ensure your joystick is connected and recognized as a `/dev/input/jsX` device. You can check with `ls /dev/input/js*`.
2.  Install `joy_linux` and `joy_teleop` (if not already installed via rosdep during colcon build):
    ```bash
    sudo apt update
    sudo apt install ros-humble-joy ros-humble-joy-linux ros-humble-joy-teleop
    ```
3.  Launch the joystick driver:
    ```bash
    ros2 launch xpi_inputs joystick.launch.py
    ```

## 📡 Interface
### Publishers
*   `/joy` (`sensor_msgs/Joy`): Publishes raw joystick button and axis states.

### Parameters
*   `dev` (string, default: `/dev/input/js0`): Path to the joystick device.
*   `deadzone` (float, default: `0.05`): Ignore small joystick movements.
*   `autorepeat_rate` (float, default: `20.0`): Rate at which events are repeated when held.

## ✅ Verification
1.  Launch the driver as described above.
2.  In a new terminal, monitor the `/joy` topic:
    ```bash
    ros2 topic echo /joy
    ```
3.  Move the joysticks and press buttons; you should see output in the terminal.

## ⚠️ Troubleshooting
*   **No `/dev/input/js0`?** Check if the joystick is connected. Try `lsusb` to see if the system detects it.
*   **Permissions error?** You might need to add your user to the `input` group: `sudo usermod -a -G input $USER`. Log out and back in for changes to take effect.
*   **Wrong device?** If you have multiple joysticks or other input devices, `/dev/input/js0` might not be the correct one. Use `js_test /dev/input/jsX` to test.
