# XPI-Blocks: Input Devices Implementation Plan

This document outlines the strategy for integrating various input devices with ROS2 on Raspberry Pi within the `xpi-blocks` project.

## 1. General Principles for Input Devices

*   **Standard ROS2 Interfaces:** Whenever possible, input devices should publish data using standard ROS2 message types (e.g., `geometry_msgs/Twist` for velocity commands, `sensor_msgs/Joy` for joysticks, custom messages for raw RC channels).
*   **Remapping & Namespacing:** Nodes must be easily remappable to avoid conflicts.
*   **Configurability:** All device-specific parameters (e.g., UART port, baud rate, GPIO pins, button mappings) must be ROS parameters.
*   **Mocking Support:** Input devices should ideally support a mock mode for testing without physical hardware, similar to our I2C and GPIO drivers.
*   **Tier 1 Nodes (Simple):** Most input devices will likely be Tier 1 nodes, focusing on publishing data. Complex processing (e.g., mapping joystick to differential drive) might be separate nodes.

## 2. Device-Specific Implementation Details

### 2.1. Keyboard Input (Teleoperation)

*   **Purpose:** Basic robot teleoperation (move forward/backward, turn).
*   **ROS2 Package:** `xpi_inputs`
*   **Approach:** Leverage existing `teleop_twist_keyboard` package or similar. Our block will provide:
    *   A wrapper/launch file for `teleop_twist_keyboard` with common key mappings.
    *   A simplified example of how to use it with our output actuators (e.g., PCA9685 controlled motor drivers).
    *   *Consideration:* `teleop_twist_keyboard` requires a terminal with focus. This is fine for basic tests.
*   **Output:** `geometry_msgs/Twist` on `/cmd_vel`.

### 2.2. Joystick / Gamepad

*   **Purpose:** Fine-grained robot control, mapping buttons/axes to commands.
*   **ROS2 Package:** `xpi_inputs`
*   **Approach:** Utilize `joy_linux` or `joy_node` from `ros_drivers`. Our block will provide:
    *   Launch files for common joysticks (e.g., Xbox 360 controller).
    *   Examples of `joy_teleop` configuration to map joystick inputs to `geometry_msgs/Twist` or other custom commands.
    *   *Hardware:* Standard USB/Bluetooth joysticks.
*   **Output:** `sensor_msgs/Joy` (raw joystick data), and potentially `geometry_msgs/Twist` via a teleop mapping node.

### 2.3. RC Remote Control (Radio Control)

This is the most complex category due to diverse protocols and hardware interfaces.

*   **Purpose:** Remote control of robotics platforms via hobbyist RC equipment.
*   **ROS2 Package:** `xpi_inputs`
*   **Supported Protocols (Initial Focus):**
    *   **SBUS:** Popular, typically UART (Serial). Provides multiple channels over a single wire.
    *   **PPM:** Older, simpler, often GPIO timing-based. Less common now.
    *   **CRSF (Crossfire / ELRS):** Modern, high-performance, typically UART/Serial.
*   **Hardware Interface:**
    *   **UART:** For SBUS, CRSF. Requires configuring Raspberry Pi's UART (disabling console login on `ttyS0`).
    *   **GPIO:** For PPM (timing analysis).
*   **Implementation Challenges:**
    *   Decoding protocols often involves precise timing or byte parsing.
    *   Handling UART permissions and baud rates.
*   **Approach:**
    *   Write dedicated Python nodes for each protocol that read from the respective interface.
    *   Publish decoded channel data (e.g., `sensor_msgs/Joy` or a custom `xpi_interfaces/msg/RCChannels`).
    *   Provide clear documentation on wiring and Raspberry Pi UART configuration.
*   **Output:** `sensor_msgs/Joy` (for compatibility) or custom `xpi_interfaces/msg/RCChannels`.

## 3. Next Steps (Implementation Order)

1.  **RC (SBUS):** Requires UART configuration and decoding logic. (Complex, but high value)
2.  **RC (CRSF):** Similar to SBUS, may share decoding logic.
3.  **RC (PPM):** GPIO timing, can be tricky.
