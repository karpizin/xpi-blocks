# XPI-Blocks: Next Key Tasks

This document outlines important tasks that require further attention or clarification before implementation.

## 1. VLM Integration for Scene Understanding (HIGH PRIORITY)

**Task:** Develop examples demonstrating how to use Vision-Language Models (VLM) for scene understanding and intelligent actuator control.

**Details:**
*   **Scene Analysis:**
    *   Use VLM for scene understanding from camera feeds (e.g., "identify objects," "detect obstacles").
    *   Integrate VLM output with control logic.
*   **Actuator Control:**
    *   Implement "tool calling" mechanism where LLM can invoke ROS2 services/actions to control actuators (e.g., "move forward", "turn left", "open gripper").
    *   Model-Controller-Perception (MCP) loops: LLM as the high-level planner/reasoner.
*   **Possible Tools:** OpenAI GPT-4o, Google Gemini 3.0 Flash, local LLMs (e.g., with Ollama).

## 2. RC Mapper / Interpreter Node (HIGH PRIORITY)

**Task:** Create a ROS2 node that maps raw `sensor_msgs/Joy` input from RC receivers (SBUS, CRSF, PPM) into more semantic or device-specific commands.

**Details:**
*   Subscribes to `sensor_msgs/Joy` messages (e.g., from `/sbus_receiver/joy`).
*   Uses ROS2 parameters for channel mapping and calibration (min/max/center).
*   Publishes: `geometry_msgs/Twist` for teleoperation, or custom messages/topics for specific actuator control.

## 3. Motion Control & Kinematics Library

**Task:** Provide a ROS2-integrated library for high-level robot motion control.

**Details:**
*   Implement odometry for diff-drive and omni-drive bases.
*   Inverse kinematics for simple manipulators.
