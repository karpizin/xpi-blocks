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
    ros2 launch xpi_llm tool_calling.launch.py llm_client_type:=gemini llm_model:=gemini-3.6-flash
    ```
*   **Local (Ollama or llama.cpp; Qwen 3.5, Gemma 4):**
    ```bash
    ros2 launch xpi_llm tool_calling.launch.py llm_client_type:=ollama llm_model:=qwen3.5:4b llm_think:=off
    # or llama.cpp: llama-server -hf ggml-org/gemma-4-E2B-it-GGUF:Q4_0 --jinja
    ros2 launch xpi_llm tool_calling.launch.py llm_client_type:=llamacpp
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
    *   *Note:* relay `0` is `/relay_node/cmd` (a `relay_node` started with `ros2 run`); relay `N` is
        `/relay_node_N/cmd`, so start more relays with `--ros-args -r __node:=relay_node_N`.

2.  **`set_pwm_channel(channel_id: int, duty_cycle: float)`**
    *   **Description:** Sets the PWM duty cycle for a specific channel on the PCA9685 driver.
    *   **Arguments:**
        *   `channel_id`: The channel ID on the PCA9685 (0-15).
        *   `duty_cycle`: Normalized PWM duty cycle from `0.0` (off) to `1.0` (full on). For servos, typical values are `0.05` to `0.10`.
    *   Only that channel changes: the other channels are sent as `NaN` ("leave unchanged" for
        `pca9685_node`), so servos and LEDs on other channels keep their position.

### Argument validation
Tool calls are checked in code, not trusted: `relay_id`/`channel_id` must be whole numbers 0-15
(`5`, `5.0` from Gemini and `"5"` from Ollama are accepted, `5.5` is not), `state` must be a
boolean (`"true"`/`"false"` accepted), `duty_cycle` a finite number 0.0-1.0. A call that fails
validation does nothing and the reason is published on `~/response`.

Commands are handled one at a time, in the order they arrive (at most 4 waiting; more are
refused with a "Busy" response).

### Parameters
*   `llm_client_type` (string, default: `gemini`): LLM backend: `gemini`, `ollama`, `llamacpp`, `openai`, `qwen`, `openrouter`. See [docs/LOCAL_LLM.md](../../../docs/LOCAL_LLM.md).
*   `llm_api_key` (string, default: `''`): API key for cloud LLMs. Empty: `GEMINI_API_KEY` or
    `OPENROUTER_API_KEY` from the environment (preferred: a parameter is visible in `ros2 param get`).
*   `llm_model` (string, default: `''`): Model; empty = the backend's default (`gemini-3.6-flash`, Ollama `qwen3.5:4b`). E.g. `gemma4:e4b`, `qwen3.5:9b`, `gemma-4-26b-a4b-it`.
*   `ollama_host` (string, default: `http://localhost:11434`): URL of your Ollama server (used when `llm_base_url` is empty).
*   `llm_base_url` (string, default: `''`): Server URL for `ollama`, `llamacpp`, `openai`, `qwen`; empty = the backend's default.
*   `llm_think` (string, default: `auto`): Ollama reasoning for Qwen 3.x / Gemma 4: `auto`, `on`, `off` (`off` is faster).
*   `llm_temperature` (float, default: `0.1`): Lower for more reliable tool calls.
*   `llm_max_tokens` (int, default: `200`): Maximum length of LLM's response.
*   `command_topic` (string, default: `~/command`): ROS2 topic to send text commands.
*   `response_topic` (string, default: `~/response`): ROS2 topic for LLM responses.
*   `pwm_topic` (string, default: `/pwm_driver/cmd`): PCA9685 driver topic (`/pwm_driver/cmd` with
    `pca9685.launch.py`, `/pca9685_node/cmd` with `ros2 run`).

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
