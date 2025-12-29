# XPI-Blocks: Audio AI Strategy

Choosing the right model for audio processing depends on your latency, privacy, and complexity requirements.

## 🏆 Recommended Models (Late 2025)

| **Speech-to-Text (STT)** | **Whisper (faster-whisper)** | `whisper_stt_node` | ✅ Implemented. High accuracy. Private. |
| **Text-to-Speech (TTS)** | **Piper** | `piper_tts_node` | ✅ Implemented. Fast, neural voices. |
| **Scene Understanding** | **Gemini 2.0 Flash** | `audio_analyzer_node` | Multimodal. High latency. |
| **Real-time Alerting** | **YAMNet** | `edge_audio_classifier` | Sub-millisecond. Limited categories. |

---

## 🛠 Integration Patterns

### 1. Hybrid Analysis (Cloud + Edge)
*   **Edge:** Use `audio_level_node` to monitor dB levels constantly.
*   **Cloud:** When a "High Urgency" event is detected (e.g. scream or glass break), send the clip to **Gemini 2.0 Flash** for a full report and robot action plan.

### 2. Private Assistant
*   Run `faster-whisper` on Raspberry Pi 5.
*   Process commands locally using a small LLM (e.g. `Llama-3-8B` via Ollama).
*   No data leaves the robot.

### 3. Safety Monitor
*   Use a dedicated YAMNet node to listen for specific sounds like "Smoke Alarm" or "Baby Crying" with 99% uptime and zero latency.
