# XPI-Blocks: Next Key Tasks

This document outlines important tasks that require further attention or clarification before implementation.

## 1. RC Mapper / Interpreter Node (HIGH PRIORITY)

**Task:** Create a ROS2 node that maps raw `sensor_msgs/Joy` input from RC receivers (SBUS, CRSF, PPM) into more semantic or device-specific commands.

**Details:**
*   Subscribes to raw joy messages.
*   Maps channels to semantic names (e.g., `throttle`, `yaw`).
*   Publishes `geometry_msgs/Twist`.

## 2. Motion Control & Kinematics Library (HIGH PRIORITY)

**Task:** Provide a ROS2-integrated library for high-level robot motion control.

**Details:**
*   Implement odometry for diff-drive and omni-drive bases.
*   Inverse kinematics for simple manipulators.

## 3. MPU6050 (I2C) IMU Integration

**Task:** Develop a ROS2 driver for the MPU6050 6-DOF IMU sensor.

**Details:**
*   This will use the `xpi_commons` I2C helper.
*   It will involve reading accelerometer and gyroscope data, potentially applying simple filtering or integration.
*   Output will be `sensor_msgs/Imu`.
*   The implementation will reside in `xpi_sensors`.

## 4. Advanced Joystick/Gamepad Integration

**Task:** Analyze and integrate concepts from `https://github.com/karpizin/bt-gamepad-reader` and `https://github.com/karpizin/usb-gamepad-reader` for robust Bluetooth and USB gamepad support.

**Details:**
*   The goal is to improve the existing `xpi_inputs/joystick` block, especially for Bluetooth connections which can be finicky.
*   The original plan was to fetch and summarize key Python files (`bt_gamepad_reader.py`, `usb_gamepad_reader.py`) and their `README.md` files.
*   **Current Blocker:** Direct `git clone` or fetching `blob` URLs from GitHub is not possible with current agent capabilities.
*   **Resolution Required:** User needs to provide **RAW URLs** for the relevant files (e.g., `https://raw.githubusercontent.com/karpizin/bt-gamepad-reader/main/bt_gamepad_reader.py`) for analysis.


## Strategic Roadmap (High-Level Functional Ideas)

This section outlines broader functional areas that could significantly enhance the XPI-Blocks ecosystem.

1.  **ROS2 Parameter Server UI/CLI:** Develop a user interface (UI) or command-line interface (CLI) for dynamic ROS2 parameter management, allowing real-time tuning without node restarts.
2.  **Generic Sensor Data Visualization:** Create a unified tool or web interface for real-time visualization of diverse sensor data streams, going beyond basic `rqt_plot` capabilities.
3.  **Error Handling & Fault Tolerance Framework:** Implement a robust, centralized system for error detection, graceful degradation, node recovery (e.g., auto-restart), and notification of system faults. This could leverage ROS2 Lifecycle management.
4.  **Configuration Management Tool for Sensor/Actuator Specifics:** Develop a tool to simplify the creation and management of complex hardware configurations (e.g., I2C addresses, GPIO pins, calibration coefficients, JSON configs for interpreters) for different robot setups.
5.  **Motion Control & Kinematics Library:** Provide a ROS2-integrated library for high-level robot motion control, including odometry, inverse kinematics for manipulators, and path following for mobile bases.
6.  **ROS2 Bridge to Web Interface:** Integrate a web interface (e.g., using `ros2_web_bridge` or custom solutions) for remote monitoring, control, and visualization of the robot via a web browser.
7.  **Basic Navigation Stack Integration:** Provide foundational ROS2 Navigation stack examples, leveraging our sensors (Sonar, IMU) for basic mapping, localization, and autonomous movement.
8.  **Power Management / Battery Monitoring:** Implement robust monitoring of battery status, power consumption, and provide early warnings for low power states, potentially integrating with charging strategies.
9.  **Logging & Diagnostics Tools:** Enhance logging capabilities with advanced tools for long-term data analysis, event tracing, and improved debugging of complex robot behaviors.
10. **Device Tree Overlay / udev Rules Generator:** Create utilities to automatically configure Raspberry Pi OS settings (SPI, I2C, UART, 1-Wire, GPIO permissions, device tree overlays) to simplify hardware setup.