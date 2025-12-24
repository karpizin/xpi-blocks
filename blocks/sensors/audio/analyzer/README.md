# Audio Analyzer: Noise Level & Pattern Recognition

This block provides tools for monitoring environmental audio, calculating noise levels, and identifying sound patterns (speech, music, alarms, etc.) using AI.

## 📌 Features
*   **Real-time DB Meter:** Calculates sound pressure levels in decibels (dB).
*   **Pattern Recognition:** Identifies specific sounds using LLM/VLM integration (requires `xpi_llm`).
*   **Threshold Triggering:** Automatically starts analysis when noise exceeds a certain level.

## 🔌 Hardware
Works with any ALSA-compatible microphone. See the [Audio Hardware Guide](../../../../docs/AUDIO_HARDWARE_GUIDE.md) for recommended models including I2S and USB matrices.

## 🚀 Quick Start

1.  **Install Dependencies:**
    ```bash
    sudo apt-get install libportaudio2
    pip install sounddevice numpy scipy
    ```
2.  **Run Noise Level Monitor:**
    ```bash
    ros2 launch xpi_sensors audio_level.launch.py
    ```
3.  **Run Pattern Analyzer (LLM-based):**
    ```bash
    ros2 launch xpi_llm audio_pattern_analyzer.launch.py
    ```

## 📊 Topics
### Audio Level Monitor
*   `~/db` ([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html)): Sound level in Decibels.
*   `~/rms` ([std_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html)): Root Mean Square level.

### Pattern Analyzer
*   `~/classification` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)): Detected sound type (e.g., "Music", "Siren", "Cat meowing").
*   `~/summary` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)): Detailed AI description of the audio environment.
