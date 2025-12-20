# XPI-Blocks: Architectural Manifesto

This document defines the technical standards and architectural decisions for the `xpi-blocks` project.

## 1. Workspace & Package Organization
We will use a **Category-Based Package Structure** to keep dependencies clean. A user who only needs GPIO shouldn't need to install OpenCV.

```text
src/
├── xpi_interfaces/       # Custom .msg, .srv, .action files (keep minimal, prefer standard)
├── xpi_commons/          # Shared utilities, HAL base classes, mock definitions
├── xpi_inputs/           # Nodes for Joysticks, RC, Keyboards
├── xpi_sensors/          # Nodes for LiDARs, IMUs, Cameras
├── xpi_actuators/        # Nodes for Motors, Servos, Relays
└── xpi_llm/              # Nodes for LLM/VLM integration, tool calling, sensor analysis
```

## 2. Node Design Standard
We adopt a **Tiered Node Architecture**:

*   **Tier 1: Simple Blocks (Tutorials/Basic I/O)**
    *   Inherit from standard `rclpy.node.Node`.
    *   Focus: Readability and immediate results for beginners.
    *   *Example:* LED Blink, Button Press.

*   **Tier 2: Advanced Blocks (Drivers/Complex Logic)**
    *   Inherit from `rclpy.lifecycle.NodeLifecycle`.
    *   Focus: Safety, state management, and system integration.
    *   *Example:* Motor Controller (must stop on deactivate), IMU Driver.

## 3. Configuration & Parameters
*   **No Hardcoding:** All GPIO pins, I2C addresses, baud rates, and update frequencies must be declared as ROS Parameters.
*   **Parameter Descriptors:** All parameters must have a description, type, and (where applicable) min/max range defined in the code.
*   **YAML First:** Every block must include a `params.yaml` example.

## 4. Hardware Abstraction Layer (HAL)
To ensure compatibility across Raspberry Pi generations (4, 5, Zero) and allow easy mocking.

*   **Library:** We standardize on **`gpiozero`** for GPIO and **`smbus2`** (via `xpi_commons`) for I2C.
    *   *Why?* `gpiozero` provides a clean, object-oriented API for GPIO. `smbus2` offers direct I2C access. Both are wrapped with mock factories in `xpi_commons` for testing.
*   **Interface Pattern:** Nodes should utilize `gpiozero` objects or `xpi_commons.i2c_helper.get_smbus()` for hardware access.
*   **Mocking:** Both `gpiozero` and `xpi_commons.i2c_helper` support "Mock Pin Factories" / "Mock SMBus" for CI/CD and laptop development tests.

## 5. Topics & Interfaces
*   **Standard Types:** Use `sensor_msgs`, `geometry_msgs`, `std_msgs` whenever possible.
    *   *Example:* A distance sensor must publish `sensor_msgs/Range`, not a custom `float32`.
*   **Namespacing:** All topics should be remappable, but default to generic names (e.g., `~/scan`, `~/cmd_vel`).

## 6. LLM Integration Architecture
This section details how Large Language Models (LLM) and Vision-Language Models (VLM) are integrated.

*   **LLM Client Abstraction (`xpi_llm/llm_clients.py`):**
    *   Provides a unified `LLMClient` interface for different LLM providers (Google Gemini, OpenRouter, Ollama).
    *   Allows seamless switching between cloud and local models.
    *   Supports basic text generation and Tool Calling (Function Calling).
*   **Tool Calling:**
    *   LLM nodes (e.g., `xpi_llm/tool_calling_node.py`) define available "tools" (ROS2 actions, services, topics) in a structured format (JSON schema).
    *   The LLM interprets natural language commands and decides whether to invoke a tool, generating arguments.
    *   The ROS2 node then executes the corresponding ROS2 interface.
*   **Prompting Guidelines:**
    *   Prompts should be clear, concise, and include context (e.g., "You are an AI assistant analyzing robot sonar data...").
    *   For Tool Calling, instructions on expected JSON output or tool usage should be part of the prompt (especially for Ollama which requires prompt-based tool simulation).
*   **Output:** LLM analysis/responses are typically published as `std_msgs/String`.

## 8. Swarm & Mesh Architecture
This section defines how groups of robots communicate and coordinate without external infrastructure.

*   **Communication Layer (LoRa Mesh):**
    *   Powered by **Meshtastic** hardware/firmware.
    *   Integrated into ROS2 via `meshtastic_bridge_node` (Serial or TCP bridge).
    *   Responsible for low-bandwidth, long-range "Shared State" propagation.
*   **Consensus Engine:**
    *   A decentralized voting system (`ConsensusEngine`) ensures all robots agree on mission parameters (e.g., changing from SEARCH to RESCUE mode).
    *   Uses Gossip-based protocols to reach agreement without a central server.
*   **Collective Sensing:**
    *   Robots share high-level metadata about detected objects or environment maps through the Mesh.
*   **Behavioral Emergence:**
    *   Groups follow behavioral rules (Boids, Area Coverage) to act as a single entity.

## 9. Dev Environment & CI
*   **Docker:** A `devcontainer` setup is mandatory to ensure reproducible builds across OS (macOS/Windows/Linux).
*   **Linting:** Strict `ament_flake8` and `ament_pep257` compliance.
