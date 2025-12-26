# XPI-Blocks: Project Capabilities

This document lists the high-level functional capabilities of the library.

## 🌦️ Weather & Environment
*   **Full Weather Station:** Real-time monitoring of temperature, humidity, atmospheric pressure, wind speed, wind direction, and precipitation (Rain Gauge).
*   **Precision Environment:** High-accuracy indoor monitoring using sensors like **HDC1080**, SCD4x (CO2), and BME680.
*   **Air Quality Lab:** Detection of CO2, TVOC, and particulate matter (PM1.0, PM2.5, PM10).

## 🎙️ Audio Intelligence
*   **Real-time Beat Detection:** On-device analysis of live audio to detect musical beats and rhythm intensity using `aubio`.
*   **Acoustic Perception:** Determining the direction of sound sources (DOA) using 4-microphone USB arrays (ReSpeaker).
*   **Pattern Recognition:** Real-time local classification of 500+ sounds using YAMNet Edge AI.
*   **Scene Analysis:** Deep contextual understanding of audio clips and speech using Gemini 3.0 Flash.

## 💡 Advanced Lighting
*   **Reactive Visuals:** WS2812/NeoPixel effects that flash and morph in sync with live audio beats.
*   **Modular Library:** 60+ customizable effects including Physics simulations (Fire, Water), Ambient environments (Starry Night, Forest), and Rhythmic patterns (BPM-sync).
*   **Visual Feedback:** High-density status indicators, progress bars, and battery levels using LED strips and rings.

## 👁️ Visual Perception
*   **Marker Navigation:** ArUco-based precise indoor localization and marker tracking.
*   **AI Vision:** Scene interpretation, object description, and obstacle detection using multimodal VLMs.
*   **Microwave Perception:** Motion detection through non-metallic obstacles using **RCWL-0516** Doppler radar.

## 📡 Decentralized Communication
*   **Swarm Networking:** Reliable long-range communication (up to 1km+) using LoRa Mesh (Meshtastic) or HC-12 wireless serial.
*   **Consensus:** Distributed decision-making and shared state across multiple robots.
*   **Bridges:** Bi-directional gateways for MQTT, Modbus RTU, CAN Bus, and LTE/4G.

## 🎮 Universal Control
*   **Multi-Modal Input:** Control robots via Joysticks (Bluetooth/USB), Keyboards, Web UI, Telegram Bot, or Hand Gestures (MediaPipe).
*   **Semantic Mapping:** Abstracting raw hardware inputs (SBUS, CRSF, PPM) into meaningful robot commands (Twist).

## 💾 Hardware Abstraction
*   **Unified HAL:** Standardized access to GPIO (gpiozero) and I2C (smbus2) with built-in mock support for development without hardware.