# Changelog

## 0.2.0 — 2026-11-28

First tagged release. Docker image: `ghcr.io/karpizin/xpi-blocks:0.2.0`.

### ⚠️ Changes in behaviour
- **ROS 2 topics stay on the robot** (`ROS_LOCALHOST_ONLY=1` in the image and in compose). Set
  `ROS_LOCALHOST_ONLY=0` to use rviz/rqt from another machine, on a trusted network only.
- **Missing hardware stops the node** with a clear error instead of publishing simulated data.
  Simulation only with `mock_hardware:=true`.
- **Web joystick**: listens on `127.0.0.1` by default (`host:=0.0.0.0` for phones) and needs the
  access token from the log link.
- **Motor drivers stop** when commands stop for `cmd_timeout` (0.5 s): publishers must stream.
- **docker-compose** pins the image version and passes only the needed devices (no `privileged`,
  no `/dev`).
- **PCA9685**: `NaN` in `~/cmd` now leaves a channel unchanged (it used to mean full on).
- **LLM nodes**: default model `gemini-3.6-flash`; `llm_model` empty = the backend's default.

### Safety and security (#3, #6, #8, #9)
- Command watchdog and speed limits for DC motor drivers; `move_robot` of the LLM agent bounded in
  speed and duration.
- No silent fallback to mock data (`get_smbus()` and ~35 nodes); SBUS/CRSF stop publishing on a
  serial error instead of faking sticks.
- Web joystick: token, `Origin` check (cross-site WebSocket hijacking), message size limit, local
  nipplejs. BLE bridge: encrypted pairing required, robot stops on disconnect.
- LLM tools: every argument validated in code; PWM sets only one channel; `control_relay` honours
  `relay_id`; commands handled one at a time, in order.

### Fixes (#3, #7, #9, #11)
- I2C sensors (`self.bus.mock_mode` broke 10 nodes), `i2c_mux` (topic instead of a missing
  service), BLE bridge (rewritten against the real bless API), PPM receiver (pigpio), WS2812 no
  longer segfaults off-target, rclpy parameter typing (`[]` defaults), many drivers.
- Gemini tool calling (google-genai), Ollama error handling, `vlm_observer` and sonar analyzers.
- Launch files pass parameters the nodes declare (`smart_motor.launch.py` drove the wrong GPIOs);
  TB6612 `stby_pin` and single-motor setups.

### New (#10)
- LLM backends: Ollama (Qwen 3.5, Gemma 4, native tool calling), llama.cpp `llama-server`, any
  OpenAI-compatible server, Qwen on DashScope, Gemma 4 on the Gemini API; `llm_think`. See
  `docs/LOCAL_LLM.md`.

### Tests and CI (#3, #4, #5)
- ROS-free smoke test for every node (139 entry points), config and launch-parameter checks,
  "no simulated data without the mock flag", unit tests for drivers and LLM clients: 513 tests.
- CI: smoke-test job, Docker image only pushed outside PRs, rosdep layer cached (~8 min instead of
  15–17), `setuptools<80`, `mediapipe<0.10.30`.
- Releases: a `vX.Y.Z` tag publishes the image as `X.Y.Z` and `X.Y`.
