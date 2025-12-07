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

## 2. RC Mapper / Interpreter Node (HIGH PRIORITY)

**Task:** Create a ROS2 node that maps raw `sensor_msgs/Joy` input from RC receivers (SBUS, CRSF, PPM) into more semantic or device-specific commands (e.g., `geometry_msgs/Twist`, individual motor speeds/angles, percentages).

**Details:**
*   Subscribes to `sensor_msgs/Joy` messages (e.g., from `/sbus_receiver/joy`).
*   Uses ROS2 parameters (potentially loaded from YAML) for:
    *   Channel mapping (e.g., `channel_0_is_roll`, `channel_1_is_pitch`).
    *   Calibration values (min/max/center pulse widths/stick positions for each channel).
    *   Deadzones.
    *   Scaling to output types (e.g., percent, m/s, rad/s, 0-180 degrees).
*   Publishes: `geometry_msgs/Twist` for teleoperation, or custom messages/topics for specific actuator control (e.g., `/robot/roll_cmd`, `/robot/pitch_cmd`).
*   This node provides the layer between raw RC protocol data and meaningful robot commands, addressing the "left_58%" type of interpretation.


