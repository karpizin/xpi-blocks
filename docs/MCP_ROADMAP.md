# XPI-Blocks: MCP Development Roadmap

This document outlines the strategic path to full **Model Context Protocol (MCP)** integration across the entire library. Our goal is to make every hardware capability accessible to LLMs as a Resource or a Tool.

## 🚀 Phase 1: Foundation (COMPLETED)
- [x] **MCP Client Abstraction:** Unified interface for Gemini 3.0, OpenRouter, and Ollama.
- [x] **Core Resource Aggregation:** Real-time context from CO2, Temperature, Humidity, and Noise sensors.
- [x] **Basic Tooling:** Control for Relays, TFT Displays, and WS2812B LED effects.
- [x] **System Architecture:** Defined standard for `resource://` and `tool://` patterns.

## 🌡️ Phase 2: Complete Sensing (Environmental Awareness)
*Goal: Every sensor in the library must expose its data as an MCP Resource.*
- [x] **Light & UV Resources:** Integrated BH1750, MAX44009, and VEML6070.
- [x] **Air Quality Context:** Added PM2.5 (PMS5003) and TVOC (CCS811) resources.
- [ ] **Physical Context:** Add IMU (MPU6050) for tilt/orientation and Magnetometer (QMC5883L) for heading.
- [x] **Power Context:** Added INA219 resources for battery and consumption monitoring.

## ⚙️ Phase 3: Total Interaction (Actuator Mastery)
*Goal: Every actuator must be a callable MCP Tool.*
- [x] **Motion Tools:**
    *   `move_robot(speed, duration)`: Basic support for dual motor bases.
- [ ] **Voice Tools:**
    *   `speak_phrase(text)`: Integration with Piper/Espeak TTS.
- [ ] **Advanced Visuals:**
    *   `render_ui_template(template_id, data)`: Complex JSON-based drawing on TFT.
    *   `set_expression(expression_name)`: Using the Expression Engine.

## 👁️ Phase 4: Perception & Localization (Advanced Context)
*Goal: High-level semantic context for autonomous missions.*
- [x] **Visual Resources:**
    *   `get_scene_description()`: Snapshot analysis using VLM (Gemini 1.5 Flash) via `get_visual_update` tool.
- [ ] **Navigation Resources:**
    *   `get_location()`: GPS/GNSS and UWB/BLE indoor coordinates.
    *   `get_navigation_status()`: Distance to waypoints, obstacle proximity.
- [ ] **Acoustic Context:**
    *   `identify_sound_event()`: Triggered by YAMNet local classification.

## 📡 Phase 5: Swarm & Intelligence (The Collective Brain)
*Goal: Decentralized MCP and autonomous loop closure.*
- [ ] **Mesh Resources:**
    *   `get_swarm_state()`: Context from other robots via Meshtastic LoRa Mesh.
- [ ] **Consensus Tools:**
    *   `propose_mission_change(new_mode)`: Triggering swarm-wide consensus.
- [ ] **Autonomous Monitoring Loops:**
    *   "Sentry Mode": AI-driven periodic polling of sensors without user input.
    *   "Self-Preservation": Autonomous tool calls based on power/thermal resources.
- [ ] **Local MCP Host:**
    *   Running Ollama locally on RPi 5 for 100% private, offline MCP server.

---

## 🛠 Integration Progress Matrix

| Category | ROS2 Block | MCP Type | Status |
| :--- | :--- | :--- | :--- |
| Environment | SCD4x | Resource | ✅ |
| Environment | AHT20 | Resource | ✅ |
| Audio | Level/YAMNet | Resource | ✅ |
| Power | INA219 | Resource | ⏳ Planned |
| Motion | TB6612/ESC | Tool | ⏳ Planned |
| Visual | ST7789 | Tool | ✅ |
| Interaction | Robot Face | Tool | ⏳ Planned |
| Swarm | Meshtastic | Resource | ⏳ Planned |
