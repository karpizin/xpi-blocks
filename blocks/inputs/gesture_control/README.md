# Gesture Control (MediaPipe)

This block enables you to control your robot using hand gestures captured by a camera. It uses Google's MediaPipe Hands for robust, real-time skeleton tracking (CPU-optimized).

![Hand Gestures](https://mediapipe.dev/images/mobile/hand_tracking_3d_android_gpu.gif)

## 🎯 Features
*   **3 Modes:**
    *   **Proportional:** Virtual Joystick. Hand position controls speed/turn. (Safe Deadman Switch logic).
    *   **Discrete:** Static gestures trigger fixed commands (Stop, Forward, Left...).
    *   **Joy Publisher:** Publishes raw hand coordinates and gesture flags to `sensor_msgs/Joy` for custom mapping.
*   **Recognized Gestures:** FIST, OPEN (Palm), POINTING (Index), THUMB_UP.

## 🚀 Setup

### 1. Requirements
*   **Camera:** USB Webcam or CSI Camera running a ROS2 node (e.g., `v4l2_camera`, `usb_cam`).
*   **Dependencies:** `mediapipe`, `opencv-python`.

### 2. Launch
```bash
# First, start your camera (if not already running)
ros2 run v4l2_camera v4l2_camera_node

# Start Gesture Control
ros2 launch xpi_inputs gesture.launch.py
```

### 3. Usage (Default: Proportional Mode)
1.  Show your hand to the camera.
2.  Make a **FIST** ✊ to activate control ("Deadman Switch").
3.  Move your fist **UP** (in the image) to drive Forward.
4.  Move **LEFT/RIGHT** to Turn.
5.  Open your hand ✋ to **STOP** immediately.

## ⚙️ Configuration

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `mode` | `proportional` | `proportional`, `discrete`, or `joy`. |
| `image_topic` | `/image_raw` | Topic to subscribe to. |
| `activation_gesture` | `FIST` | Gesture required to move in proportional mode. |
| `scale_linear` | `0.5` | Max speed (m/s). |

## 🛠 Joy Mode Mapping
In `joy` mode, the node publishes to `/gesture_ctl/joy`:
*   **Axis 0:** Hand X (-1.0 Left ... 1.0 Right)
*   **Axis 1:** Hand Y (-1.0 Bottom ... 1.0 Top)
*   **Button 0:** FIST detected
*   **Button 1:** OPEN detected
*   **Button 2:** POINTING detected
*   **Button 3:** THUMB_UP detected

You can use this with `joy_mapper_node` to map gestures to arbitrary robot actions (e.g., Thumb Up -> Turn on Lights).
