# AI Agent: Model Context Protocol (MCP) Host

The MCP Agent is the "brain" of the robot. It implements the **Model Context Protocol**, serving as a bridge between high-level reasoning (LLM) and low-level hardware (ROS2).

Unlike simple command executors, the MCP Agent is **context-aware**: it continuously monitors sensor data (Resources) and includes this information in every request to the LLM.

## 🧠 Features
*   **Real-time Context:** Injects current temperature, CO2 levels, noise, and battery status into the system prompt.
*   **Tool Orchestration:** Allows the LLM to autonomously decide when to use actuators (e.g., "It's too stuffy here, I'll turn on the fan").
*   **Gemini 3.0 Integration:** Optimized for the latest multimodal models with high reasoning capabilities.

---

## 🛰️ Resources (Input Context)
The agent automatically subscribes to the following topics to build its understanding of the environment:

| Resource | ROS2 Topic | Description |
| :--- | :--- | :--- |
| `co2` | `/scd4x/co2` | Carbon Dioxide concentration (ppm). |
| `noise_db` | `/audio_level/db` | Environmental noise level in decibels. |
| `temp` | `/aht20/temperature` | Ambient temperature (°C). |
| `hum` | `/aht20/humidity` | Relative humidity (%). |

---

## 🛠️ Tools (Actuator Control)
The LLM can invoke these functions based on the user's request or the current context:

### 1. `control_relay`
Turns a physical relay ON or OFF. 
*   **Use case:** Controlling fans, lights, or pumps.
*   **Arguments:** `relay_id` (0-15), `state` (true/false).

### 2. `display_text`
Writes a message to the robot's color TFT display.
*   **Arguments:** `text` (string), `color` (string).

### 3. `display_alert`
Shows a high-priority warning on the screen with a red background.
*   **Arguments:** `message` (string).

### 4. `set_led_effect`
Changes the visual effect of the WS2812B RGB LED strip.
*   **Arguments:** `effect_id` (integer, 0-100).

### 5. `clear_display`
Wipes the screen to a solid color.
*   **Arguments:** `color` (string, optional).

---

## 🚀 Usage

### 1. Launch the Agent
```bash
ros2 launch xpi_llm mcp_agent.launch.py api_key:="YOUR_GEMINI_API_KEY"
```

### 2. Send Natural Language Commands
```bash
# General query (Agent will use CO2 resource to answer)
ros2 topic pub --once /mcp_agent/command std_msgs/String "data: 'How is the air quality here?'"

# Complex command involving tools
ros2 topic pub --once /mcp_agent/command std_msgs/String "data: 'If it is too loud, show a warning on the screen and turn on the red light (relay 0)'"
```

---

## 📝 Example Interaction
**User:** "Is it comfortable in the room?"

**Agent Logic:**
1. Sees `temp: 28.5`, `hum: 65%`, `co2: 1200`.
2. Decides that it's too hot and stuffy.
3. Calls `control_relay(relay_id=1, state=true)` to start ventilation.
4. Calls `display_text(text="Ventilating...", color="yellow")`.
5. **Response:** "It's a bit stuffy (CO2 is 1200ppm). I've started the ventilation for you."
