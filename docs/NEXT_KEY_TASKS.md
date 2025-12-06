# XPI-Blocks: Next Key Tasks

This document outlines important tasks that require further attention or clarification before implementation.

## 1. LLM/VLM Integration for Smart Control and Analysis (HIGH PRIORITY)

**Task:** Develop examples demonstrating how to use Large Language Models (LLM) and Vision-Language Models (VLM) for advanced sensor data analysis (filtering, anomaly detection, trend identification) and intelligent actuator control (via tool calling or Model-Controller-Perception loops).

**Details:**
*   **Sensor Analysis:**
    *   Feed sensor data streams (e.g., IMU, range, vision) to LLMs for high-level interpretation.
    *   Anomaly detection in sensor readings using LLM's pattern recognition.
    *   Trend analysis (e.g., "Is the temperature consistently rising?")
*   **Actuator Control:**
    *   Implement "tool calling" mechanism where LLM can invoke ROS2 services/actions to control actuators (e.g., "move forward", "turn left", "open gripper").
    *   Model-Controller-Perception (MCP) loops: LLM as the high-level planner/reasoner.
*   **VLM Integration:**
    *   Use VLM for scene understanding from camera feeds (e.g., "identify objects," "detect obstacles").
    *   Integrate VLM output with control logic.
*   **Challenges:** Latency, computational requirements, safety (for actuator control).
*   **Possible Tools:** OpenAI APIs, Google Gemini API, local LLMs (e.g., with Ollama), vision APIs.

## 2. Advanced Joystick/Gamepad Integration

**Task:** Analyze and integrate concepts from `https://github.com/karpizin/bt-gamepad-reader` and `https://github.com/karpizin/usb-gamepad-reader` for robust Bluetooth and USB gamepad support.

**Details:**
*   The goal is to improve the existing `xpi_inputs/joystick` block, especially for Bluetooth connections which can be finicky.
*   The original plan was to fetch and summarize key Python files (`bt_gamepad_reader.py`, `usb_gamepad_reader.py`) and their `README.md` files.
*   **Current Blocker:** Direct `git clone` or fetching `blob` URLs from GitHub is not possible with current agent capabilities.
*   **Resolution Required:** User needs to provide **RAW URLs** for the relevant files (e.g., `https://raw.githubusercontent.com/karpizin/bt-gamepad-reader/main/bt_gamepad_reader.py`) for analysis.

## 2. RC (CRSF) Receiver Implementation

**Task:** Implement a ROS2 driver for CRSF protocol data from RC receivers via UART.

**Details:**
*   Similar to SBUS, this will involve UART configuration and parsing a binary protocol.
*   The implementation will likely reside in `xpi_inputs`.

## 3. RC (PPM) Receiver Implementation

**Task:** Implement a ROS2 driver for PPM protocol data from RC receivers via GPIO timing.

**Details:**
*   This will involve precise GPIO timing analysis, potentially using `gpiozero`'s capabilities or a dedicated library.
*   The implementation will likely reside in `xpi_inputs`.

## 4. MPU6050 (I2C) IMU Integration

**Task:** Develop a ROS2 driver for the MPU6050 6-DOF IMU sensor.

**Details:**
*   This will use the `xpi_commons` I2C helper.
*   It will involve reading accelerometer and gyroscope data, potentially applying simple filtering or integration.
*   Output will be `sensor_msgs/Imu`.
*   The implementation will reside in `xpi_sensors`.

## 5. TB6612FNG (GPIO) Motor Driver Implementation

**Task:** Develop a ROS2 driver for the TB6612FNG dual DC motor driver.

**Details:**
*   This will involve controlling motor direction (GPIO) and speed (PWM via GPIO).
*   Output will be `geometry_msgs/Twist` or custom motor commands.
*   The implementation will reside in `xpi_actuators`.