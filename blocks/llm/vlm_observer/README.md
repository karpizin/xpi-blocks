# VLM Observer

This block integrates a **Vision-Language Model (VLM)** to give your robot "sight" and "understanding". It periodically captures an image from the robot's camera, sends it to an AI model (Gemini, GPT-4o, or local Llava), and publishes a semantic description of the scene.

## 🧠 Use Cases
*   **Scene Description:** "I see a living room with a sofa and a cat on the floor."
*   **Object Detection:** "List all tools visible on the table."
*   **Hazard Detection:** "Is there any water spilled on the floor?"
*   **Contextual Logic:** "Is the door open or closed?"

## ⚡ Prerequisites
1.  **Camera Block:** You must have a working camera publishing to `/camera/image_raw`. (See `blocks/sensors/camera`).
2.  **API Key:**
    *   **Google Gemini:** Get a free key from [Google AI Studio](https://aistudio.google.com/).
    *   **OpenRouter:** For access to GPT-4o/Claude/etc.
    *   **Ollama:** For local processing (requires powerful hardware/Jetson/NPU, might be slow on RPi CPU).

## 🚀 Usage

### 1. Export API Key
For security, use environment variables.
```bash
export GEMINI_API_KEY="your_key_here"
# or
export OPENROUTER_API_KEY="your_key_here"
```

### 2. Launch Camera (if not running)
```bash
ros2 launch xpi_sensors camera.launch.py
```

### 3. Launch VLM Observer
```bash
ros2 launch xpi_llm vlm_observer.launch.py provider:=gemini interval:=10.0
```

### 4. View Results
```bash
ros2 topic echo /vlm/detected_objects
```

## ⚙️ Parameters

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `image_topic` | `/camera/image_raw` | The ROS topic to listen to. |
| `analysis_interval` | `30.0` | How often (seconds) to send an image to the AI. |
| `provider` | `gemini` | `gemini`, `openrouter`, or `ollama`. |
| `model` | `gemini-1.5-flash` | Specific model name (e.g., `gpt-4o`). |
| `prompt` | *Generic description* | The question you want the AI to answer about the image. |

## ⚠️ Considerations
*   **Latency:** Cloud APIs take 1-3 seconds to respond. This is not for real-time obstacle avoidance (use Lidar/Sonar for that).
*   **Cost:** Gemini Flash is currently free (within limits). GPT-4o costs money per image.
*   **Privacy:** Images are sent to the cloud. Be mindful of where you point the camera.
