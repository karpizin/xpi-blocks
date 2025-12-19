# Robot Face: Multi-Display Facial Expressions

This block provides a versatile system for rendering robot facial expressions (primarily "eyes"). It supports 50+ emotional states and scales perfectly from 16x16 LED matrices to high-resolution LCDs.

## 🎭 Features
*   **Procedural Rendering:** Eyes are drawn using geometry, allowing smooth transitions and infinite scaling.
*   **50+ Emotions:** Defined via parameters (eye shape, lid tilt, pupil size).
*   **Animation Support:** Smooth interpolation between states (e.g., Neutral -> Happy -> Blink).
*   **Multi-Display Output:**
    *   **High-Res:** Beautiful anti-aliased eyes for HDMI/DSI screens.
    *   **OLED:** Sharp 1-bit eyes for SSD1306.
    *   **LED Matrix:** 16x16 or 8x8 pixel-art style eyes.

## 🚀 Usage

### 1. Launch the Engine and a Simulator
```bash
ros2 launch xpi_hci robot_face.launch.py
```

### 2. Trigger an Emotion
```bash
ros2 topic pub /robot_face/set_expression std_msgs/msg/String "{data: 'ANGRY'}" -1
```

## ⚙️ Supported Emotions (Excerpt)
`NEUTRAL`, `HAPPY`, `ANGRY`, `SAD`, `THINKING`, `SURPRISED`, `SLEEPY`, `WINK`, `EVIL`, `CURIOUS`, `CONFUSED`, and 40+ more.

## 🛠 Tech Stack
*   **Backend:** Python `Pillow` (PIL) for drawing.
*   **Interpolation:** `numpy` for smooth frame transitions.
*   **Standard:** Publishes `sensor_msgs/Image`.
