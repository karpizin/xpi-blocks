# Local Audio Classifier: YAMNet (Edge AI)

YAMNet is a deep net that predicts 521 audio event classes based on the [AudioSet](https://research.google.com/audioset/index.html) ontology. It runs locally using TensorFlow Lite, making it ideal for real-time sound detection (e.g., siren, bark, glass break) without cloud latency.

## 📌 Features
*   **521 Classes:** Recognizes everything from "Police Siren" to "Cat Meowing".
*   **Zero Latency:** Runs entirely on the Raspberry Pi CPU.
*   **Privacy:** No audio data is sent to the cloud.

## 🔌 Hardware
Compatible with any microphone (USB or I2S) supported by `audio_level_node`.

## 🚀 Quick Start

1.  **Install Dependencies:**
    ```bash
    pip install tflite-runtime numpy resampy sounddevice
    ```
2.  **Download Model:**
    The node will automatically download the `.tflite` model and class map on the first run.
3.  **Run the Node:**
    ```bash
    ros2 launch xpi_sensors yamnet.launch.py
    ```
4.  **Listen for Events:**
    ```bash
    ros2 topic echo /yamnet/detections
    ```

## 📊 Topics
*   `~/detections` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)): Top detected class name.
*   `~/raw_detections` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)): JSON string with top-3 classes and confidence scores.
