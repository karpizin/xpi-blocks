# Future Input Devices & Methods

This document expands the scope of "Input" beyond standard joysticks and keyboards. It outlines potential future blocks to make robot interaction more diverse, natural, and embedded.

## 1. Tactile & Embedded Inputs (Physical UI)
*Inputs mounted directly on the robot chassis for configuration and manual override.*

### A. Rotary Encoder (EC11)
*   **Description:** An infinite-rotation knob with tactile clicks and a push button.
*   **Use Cases:**
    *   Navigating OLED menus (Scroll / Select).
    *   Adjusting PID PID coefficients on the fly.
    *   Volume / Brightness control.
*   **Implementation:**
    *   GPIO interrupts (via `gpiozero.RotaryEncoder`).
    *   Output: `Int32` (Counter), `Int32` (Delta), `Bool` (Button).

### B. Matrix Keypad (4x4 or 3x4)
*   **Description:** Membrane keypad often used in security systems.
*   **Use Cases:**
    *   Entering PIN codes to unlock the robot.
    *   Selecting modes (Press 'A' for Patrol, 'B' for Sleep).
*   **Implementation:**
    *   Scanning GPIO matrix (row/col).
    *   Output: `std_msgs/String` or custom event.

### C. Potentiometer (Analog Knob)
*   **Description:** Standard analog dial.
*   **Use Cases:** Setting max speed limit, setting servo zero-position.
*   **Implementation:** Requires ADC (ADS1115). Need a dedicated mapper to convert Voltage -> Float Parameter.

---

## 2. Kinetic & Motion Inputs
*Controlling the robot by moving a controller or your body.*

### A. Wii Nunchuk (I2C)
*   **Description:** The classic Wii controller attachment. Cheap, ergonomic, one-handed.
*   **Sensors:** 2-axis Joystick, 2 Buttons (C, Z), 3-axis Accelerometer.
*   **Why:** Connects directly to I2C (no USB dongle needed). Great for "wired remote" or embedded builds.
*   **Implementation:**
    *   I2C reading (Address 0x52).
    *   Output: `sensor_msgs/Joy` (compatible with `joy_mapper`).

### B. IMU-based Gestures
*   **Description:** Using the robot's own IMU or a wearable IMU to detect taps/shakes.
*   **Use Cases:** "Tap robot to wake up", "Shake to reset error".
*   **Implementation:** Analysis of `sensor_msgs/Imu` acceleration spikes.

---

## 3. Audio & Voice Input
*Hands-free interaction.*

### A. Voice Command Node
*   **Description:** Local speech-to-text running on the Pi.
*   **Tech:** `Vosk` (very light), `Whisper.cpp` (higher quality, heavier).
*   **Pipeline:** Microphone -> Text -> `std_msgs/String` -> LLM Tool Caller -> Robot Action.
*   **Use Cases:** "Robot, stop!", "Go to the kitchen."

### B. Audio Event Detection
*   **Description:** Detecting non-speech sounds.
*   **Use Cases:** Clapping to toggle lights, reacting to breaking glass.

---

## 4. Web & Network Inputs (BYOD)
*Using smartphones/tablets as controllers.*

### A. Virtual Joystick (Web UI)
*   **Description:** A hosted web page served by the robot (Flask/FastAPI + Rosbridge).
*   **UI:** Virtual on-screen sticks and buttons.
*   **Why:** Everyone has a phone. No app installation required.
*   **Implementation:** WebSocket -> `geometry_msgs/Twist`.

### B. Telegram/Discord Bot
*   **Description:** Controlling the robot via chat app.
*   **Use Cases:** Remote monitoring over the internet (NAT traversal handled by Telegram).
*   **Implementation:** Python bot API -> ROS2 topics.
