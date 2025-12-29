# Universal Status Indication System (USIS)

This standard defines visual signals for a single RGB LED (WS2812 or standard 4-pin RGB) used as a system indicator in XPI-Blocks projects.

## 🎨 1. Color Palette (High Contrast)

| Color | HEX Code | Meaning / Intent |
| :--- | :--- | :--- |
| **Green** | `#00FF00` | All systems nominal (System OK) |
| **Red** | `#FF0000` | Critical error (Hardware Failure) |
| **Orange** | `#FFAA00` | Warning / Low Battery |
| **Blue** | `#0000FF` | Communication Active (Bluetooth / LoRa) |
| **Cyan** | `#00FFFF` | Autonomous Mode / GPS Fixed |
| **Magenta** | `#FF00FF` | AI Thinking / Processing |
| **Yellow** | `#FFFF00` | Calibration Mode / Settings |
| **White** | `#FFFFFF` | System Boot / Initialization |

## ⚡ 2. Animation Patterns

| Pattern | Description | System State |
| :--- | :--- | :--- |
| **Solid** | Constant on | Stable operation in current mode. |
| **Breathe** | Slow fade (1-2 sec) | Standby / Sleep mode. |
| **Blink (1Hz)** | 500ms ON / 500ms OFF | Normal activity / Data transfer. |
| **Fast-Blink (5Hz)** | High frequency | Signal seeking / Pairing process. |
| **Double-Blink** | Two flashes then pause | Non-critical error (e.g., poor GPS signal). |
| **SOS / Heartbeat** | Two short, one long | Alert / Help needed. |

## 🛠 3. Error Code Specification (Visual Codes)

When errors occur, the indicator switches to **Red** mode with specific pulse counts:
*   **1 flash**: Power error / Battery Critical.
*   **2 flashes**: I2C Bus error / Sensor not found.
*   **3 flashes**: Communication Down.
*   **4 flashes**: ROS2 Error (Node Crash).
*   **5 flashes**: Actuator Error / Motor Stalled.

## 🚀 4. Software Implementation (API)

The `status_indicator_node` provides the following:
1.  Listens to the `/status/code` topic (`String` or `Int32`).
2.  Translates codes into parameters for the WS2812 driver.
3.  Implements priority logic: errors override normal status indication.

## 🛰 5. Usage Examples
*   **Drone Takeoff**: White (Blink) -> Green (Solid).
*   **Robodog Thinking (VLM)**: Magenta (Breathe).
*   **RC Connection Lost**: Orange (Fast-Blink).