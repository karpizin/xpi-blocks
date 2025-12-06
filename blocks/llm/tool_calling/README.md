# LLM Tool Calling for Actuator Control

This block enables an LLM to interpret natural language commands and execute corresponding ROS2 actions to control actuators (e.g., Relays, PCA9685 PWM channels). It uses the LLM's "function calling" or "tool calling" capabilities.

## 📦 Bill of Materials
*   Raspberry Pi with running actuator nodes (e.g., `xpi_actuators/relay_node`, `xpi_actuators/pca9685_node`).
*   Internet connection (for cloud LLMs like Gemini/OpenRouter) OR a local Ollama server running a compatible model.
*   An API key for cloud LLM providers.

## 🚀 Quick Start

### 1. Launch Actuator Nodes
Ensure your desired actuator nodes are running. For example:
*   **Relay Node (GPIO 17):** `ros2 launch xpi_actuators relay.launch.py gpio_pin:=17 name:=my_relay`
*   **PCA9685 Node:** `ros2 launch xpi_actuators pca9685.launch.py`

### 2. Prepare LLM Environment
*   **For Gemini API:** Set `GEMINI_API_KEY` environment variable.
*   **For OpenRouter API:** Set `OPENROUTER_API_KEY` environment variable.
*   **For Ollama (Local LLM):** Ensure Ollama server is running and the model is pulled (`ollama run llama2`).

### 3. Launch the LLM Tool Calling Node
Choose your desired LLM client (parameters similar to `sonar_trend_analyzer`):

*   **Using Gemini:**
    ```bash
    export GEMINI_API_KEY="YOUR_GEMINI_API_KEY"
    ros2 launch xpi_llm tool_calling.launch.py llm_client_type:=gemini llm_model:=gemini-pro
    ```
*   **Using Ollama:**
    ```bash
    ros2 launch xpi_llm tool_calling.launch.py llm_client_type:=ollama llm_model:=llama2 ollama_host:=http://localhost:11434
    ```

## 📡 Interface
### Subscribers
*   `~/command` (`std_msgs/String`): Send your natural language commands here (e.g., "Turn on relay 0", "Set servo 5 to 7.5% duty cycle").

### Publishers
*   `~/response` (`std_msgs/String`): LLM's textual response or confirmation of tool execution.

### Tools Available to the LLM
The LLM is aware of the following functions it can call:

1.  **`set_relay_state(relay_id: int, state: bool)`**
    *   **Description:** Sets the state of a specific relay connected to a GPIO pin.
    *   **Arguments:**
        *   `relay_id`: The ID of the relay to control (0-15).
        *   `state`: `true` for ON, `false` for OFF.
    *   *Note:* The `relay_id` corresponds to a dynamically remapped topic, e.g., `relay_node_0/cmd`. If you launch a relay node without `name` remapping, it will use `/relay_node/cmd` (which corresponds to `relay_id=0` in this tool).

2.  **`set_pwm_channel(channel_id: int, duty_cycle: float)`**
    *   **Description:** Sets the PWM duty cycle for a specific channel on the PCA9685 driver.
    *   **Arguments:**
        *   `channel_id`: The channel ID on the PCA9685 (0-15).
        *   `duty_cycle`: Normalized PWM duty cycle from `0.0` (off) to `1.0` (full on). For servos, typical values are `0.05` to `0.10`.

### Parameters
*   `llm_client_type` (string, default: `gemini`): Which LLM backend (`openrouter`, `gemini`, `ollama`).
*   `llm_api_key` (string, default: `''`): API key for cloud LLMs.
*   `llm_model` (string, default: `gemini-pro`): Specific model name.
*   `ollama_host` (string, default: `http://localhost:11434`): URL of your local Ollama server.
*   `llm_temperature` (float, default: `0.1`): Lower for more reliable tool calls.
*   `llm_max_tokens` (int, default: `200`): Maximum length of LLM's response.
*   `command_topic` (string, default: `~/command`): ROS2 topic to send text commands.
*   `response_topic` (string, default: `~/response`): ROS2 topic for LLM responses.

## ✅ Verification
1.  Launch the `relay_node` (or `pca9685_node`) and `llm_tool_calling_node`.
2.  Send a command to the `llm_controller`:
    ```bash
    ros2 topic pub --once /llm_controller/command std_msgs/msg/String "{data: 'Turn on relay 0'}"
    ```
    OR
    ```bash
    ros2 topic pub --once /llm_controller/command std_msgs/msg/String "{data: 'Set servo channel 5 to 7.5 percent duty cycle'}"
    ```
3.  Monitor the `~/response` topic for LLM's reply:
    ```bash
    ros2 topic echo /llm_controller/response
    ```
    And observe your actual hardware!

## ⚠️ Troubleshooting
*   **LLM not calling tools?**
    *   Ensure `llm_temperature` is low (`0.1` to `0.3`) for deterministic behavior.
    *   Check LLM logs for errors or safety blocks.
    *   For Ollama, ensure the model you're using is capable of following instructions to output JSON. Fine-tuned models might perform better.
*   **Tool call received, but actuator not responding?**
    *   Verify the actuator node (e.g., `relay_node`) is running and subscribing to the correct topic (`/relay_node_0/cmd` for relay 0).
    *   Check for errors in the `llm_controller` node's logs regarding tool execution.
