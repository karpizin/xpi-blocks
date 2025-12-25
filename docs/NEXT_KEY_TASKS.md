# XPI-Blocks: Next Key Tasks

This document outlines important tasks that require further attention or clarification before implementation.

## 1. VLM Integration for Scene Understanding (COMPLETED)

**Task:** Develop examples demonstrating how to use Vision-Language Models (VLM) for scene understanding and intelligent actuator control.

**Details:**
*   **Scene Analysis:**
    *   Implemented `VLMObserverNode` with trigger support.
    *   Integrated VLM context into `MCPAgentNode`.
*   **Actuator Control:**
    *   Implemented "tool calling" mechanism where LLM can invoke ROS2 services/actions to control actuators.
    *   Added `get_visual_update` tool to `MCPAgentNode`.

## 2. RC Mapper / Interpreter Node (COMPLETED)

**Task:** Create a ROS2 node that maps raw `sensor_msgs/Joy` input from RC receivers (SBUS, CRSF, PPM) into more semantic or device-specific commands.

**Details:**
*   Implemented `RCInterpreterNode` in `xpi_inputs`.
*   Supports channel mapping, expo, deadzone, and switch bindings.

## 3. Motion Control & Kinematics Library

**Task:** Provide a ROS2-integrated library for high-level robot motion control.

**Details:**
*   Implement odometry for diff-drive and omni-drive bases.
*   Inverse kinematics for simple manipulators.