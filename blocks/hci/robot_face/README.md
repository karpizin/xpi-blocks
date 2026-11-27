# Robot Face: Multi-Display Facial Expressions

> ⚠️ **Experimental, unfinished.** This branch holds a work-in-progress redesign (`Variant A`, see
> [DESIGN_DIRECTIONS.md](DESIGN_DIRECTIONS.md)). It is parked for later and is not ready to merge.

This block provides a versatile system for rendering robot facial expressions (primarily "eyes"). It supports 20+ emotional states and scales from 16x16 LED matrices to high-resolution LCDs, with a clean companion-style visual language optimized for dark displays.

The repository currently implements the more restrained `Variant A` design direction. A product/design comparison between `Variant A` and `Variant B` lives in [DESIGN_DIRECTIONS.md](DESIGN_DIRECTIONS.md).

## 🎭 Features
*   **Procedural Rendering:** Eyes are drawn using geometry, allowing smooth transitions and infinite scaling.
*   **Eyes-First Visual Style:** Expressions are driven mostly by gaze, lids, asymmetry, and rim intensity, with the default face remaining mouthless.
*   **20+ Emotions:** Defined via compact parameter vectors, while the active renderer maps them into a minimal eyes-and-rim system.
*   **Animation Support:** Smooth interpolation and soft idle motion instead of hard, cartoon-like pose jumps.
*   **Display Pipeline:**
    *   **High-Res Render:** The engine publishes anti-aliased RGB face frames as `sensor_msgs/Image`.
    *   **LED Matrix Adapter:** The package includes a generic adapter that resizes the face image for 16x16 or 8x8 displays.
    *   **Other Displays:** OLED, TFT, or HDMI outputs can consume the published image stream directly or through a dedicated adapter node.

## 🚀 Usage

### 1. Launch the Engine and a Simulator
```bash
ros2 launch xpi_hci robot_face.launch.py
```

### 2. Trigger an Emotion
```bash
ros2 topic pub /robot_face/set_expression std_msgs/msg/String "{data: 'ANGRY'}" -1
```

### 3. Consume the Rendered Output
*   High-resolution face frames are published on `/robot_face/eyes_image`
*   The LED matrix adapter publishes resized grayscale values on `/robot_face_matrix/matrix_raw`

## ⚙️ Supported Emotions (Excerpt)
`NEUTRAL`, `HAPPY`, `ANGRY`, `SAD`, `THINKING`, `SURPRISED`, `SLEEPY`, `WINK`, `EVIL`, `CURIOUS`, `CONFUSED`, and more.

## 🛠 Tech Stack
*   **Backend:** Python `Pillow` (PIL) for drawing.
*   **Interpolation:** `numpy` for smooth frame transitions.
*   **Standard:** Publishes `sensor_msgs/Image`.
