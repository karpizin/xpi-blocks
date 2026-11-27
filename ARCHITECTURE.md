# XPI-Blocks: Architectural Manifesto

This document defines the technical standards and architectural decisions for the `xpi-blocks` project.

## 1. Workspace & Package Organization
We use a **Category-Based Package Structure** to minimize cross-dependencies. A user who only needs GPIO access should not be forced to install heavy dependencies like OpenCV or TensorFlow.

```text
src/
├── xpi_commons/          # Shared utilities, HAL helpers, storage blocks
├── xpi_inputs/           # Nodes for joysticks, RC, keyboards, web control
├── xpi_sensors/          # Nodes for LiDARs, IMUs, GPS, environment sensors
├── xpi_actuators/        # Nodes for motors, LEDs, displays, relays, steppers
├── xpi_comms/            # Wired/wireless bridges: serial, CAN, LoRa, BLE, LTE
├── xpi_audio/            # Speech, TTS, beat detection, audio-related nodes
├── xpi_hci/              # Human-facing interaction and expression nodes
├── xpi_llm/              # LLM/VLM integration and tool-calling nodes
├── xpi_navigation/       # SLAM, ranging, indoor positioning algorithms
├── xpi_vision/           # Vision tracking and perception nodes
├── xpi_tools/            # CLI and terminal tooling
└── xpi_projects/         # Integrated launchers and project-level composition
```

At the moment the workspace intentionally relies mostly on standard ROS message
packages. A dedicated `xpi_interfaces/` package can be added later if custom
messages, services, or actions become necessary.

## 2. Node Design Standard
We adopt a **Tiered Node Architecture** to cater to different complexity levels:

*   **Tier 1: Simple Blocks (Standard Nodes)**
    *   Inherit from `rclpy.node.Node`.
    *   Focus: Readability and immediate results for rapid prototyping.
*   **Tier 2: Advanced Blocks (Lifecycle Nodes)**
    *   Inherit from `rclpy.lifecycle.NodeLifecycle`.
    *   Focus: Deterministic state management and safety (e.g., stopping motors on node deactivation).

## 3. Configuration & Parameters
*   **No Hardcoding:** All GPIO pins, I2C addresses, and update frequencies must be declared as ROS Parameters.
*   **Parameter Descriptors:** All parameters must have a description, type, and valid range (min/max) defined in the code.
*   **YAML Configuration:** Every block must include a `params.yaml` example.

## 4. Hardware Abstraction Layer (HAL)
To ensure compatibility across Raspberry Pi generations (4, 5, Zero 2W) and allow easy simulation.

*   **Libraries:** We standardize on **`gpiozero`** for GPIO and **`smbus2`** (via `xpi_commons`) for I2C.
    *   *Why?* `gpiozero` provides a clean, object-oriented API. `smbus2` offers robust I2C access. Both support mock factories.
*   **Mocking:** All drivers must support `GPIOZERO_PIN_FACTORY=mock` for testing in CI/CD environments and on non-Pi hardware.

## 5. Topics & Interfaces
*   **Standard Types:** Use `sensor_msgs`, `geometry_msgs`, `std_msgs` whenever possible.
    *   *Example:* A distance sensor must publish `sensor_msgs/Range`, not a custom `float32`.
*   **Namespacing:** All topics should be remappable, but default to generic local names (e.g., `~/scan`, `~/cmd_vel`).

## 6. AI & LLM Integration Architecture
Integrating Large Language Models into the robotics loop.

*   **LLM Client Abstraction:** A unified interface for switching between cloud (Gemini, GPT) and local (Ollama) backends.
*   **Tool Calling:** 
    *   LLM nodes define available ROS2 services/actions as structured JSON schemas.
    *   The LLM interprets natural language commands and decides which "tool" to invoke.
*   **Prompting Guidelines:** 
    *   Prompts must include role-playing context ("You are an AI assistant for a 6-legged robot...").
    *   Explicit instructions for JSON output formats are required for local models.
*   **Context Injection:** Real-time sensor data is serialized into prompts to allow the LLM to make informed decisions.

## 7. Hexapod Master Core
A complete stack for 6-legged locomotion:
*   **Inverse Kinematics (IK)**: Analytical solution for 3-DOF legs.
*   **Gait Generation**: Dynamic support for **Tripod**, **Wave**, and **Ripple** gaits with smooth interpolation between states.
*   **Auto-Leveling**: Fusion of IMU data to stabilize the robot body on uneven terrain.

## 8. Swarm & Mesh Architecture (Decentralized)
Coordination without a central server or external infrastructure.

*   **LoRa Mesh**: Powered by **Meshtastic** for long-range, low-bandwidth communication.
*   **Consensus Engine**: A decentralized voting system for mission synchronization (e.g., all robots agreeing to change mission state based on local observations).
*   **Collective Sensing**: Sharing high-level object metadata via the mesh.

## 9. Status Indication System (USIS)
Standardized visual feedback using RGB LEDs (WS2812):
*   `Blue Pulsing`: System Boot / Idle.
*   `Green Constant`: Ready for operation.
*   `Red Blinking`: Hardware or Logic Error.
*   `Yellow Rotating`: AI Processing / Thought state.
