# XPI-Blocks: Model Context Protocol (MCP) Integration

This document defines how the project implements the **Model Context Protocol** to connect LLMs with robot hardware.

## 🧱 Concepts

### 1. Resources (Data)
Resources are entities that provide context to the LLM. In XPI-Blocks, every sensor node is a potential resource provider.
*   **Pattern:** `resource://sensors/[sensor_name]`
*   **Example:** `resource://sensors/scd4x/co2` provides the latest CO2 concentration.

### 2. Tools (Actions)
Tools are executable functions that the LLM can invoke to change the state of the robot or the environment.
*   **Pattern:** `tool://actuators/[actuator_name]`
*   **Example:** `tool://actuators/relay/set_state` toggles a physical relay.

### 3. Prompts (Templates)
Standardized ways to interact with specific robot configurations.
*   **Example:** "Security Patrol Prompt" - a system instruction that tells the LLM how to use audio and vision resources to detect intruders.

## 🛰️ Implementation: The MCP Agent Node

The `mcp_agent_node` acts as the "Host" in MCP terminology. It:
1.  **Aggregates Resources:** Collects the latest values from all active sensor topics.
2.  **Exposes Tools:** Maps ROS2 services and actions to JSON-schema tool definitions.
3.  **Maintains Context:** Injects the current "Robot State" (JSON representation of all resources) into every LLM request.

## 🚀 Benefits
*   **Model Agnostic:** Works with Gemini, GPT, or Claude without changing the robot code.
*   **Self-Documenting:** The robot tells the model what it can do and what it sees.
*   **Real-time Context:** The model always makes decisions based on the latest sensor telemetry.

---

## 📈 Future
See the [**MCP Development Roadmap**](MCP_ROADMAP.md) for the full integration plan.
