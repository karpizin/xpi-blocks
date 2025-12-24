# Sound Source Localization: ReSpeaker Mic Array v2.0

The ReSpeaker Mic Array v2.0 is a hardware-based solution for sound source localization (SSL) and voice interaction. It features 4 high-performance microphones and an onboard DSP (XMOS) that handles Beamforming, AEC, and DOA (Direction of Arrival) estimation.

## 📌 Features
*   **DOA (Direction of Arrival):** Provides the angle (0-359°) of the detected sound source.
*   **VAD (Voice Activity Detection):** Detects if the sound is human speech.
*   **Beamforming:** Focuses on the sound source while suppressing background noise.
*   **Interface:** USB (Plug & Play).

## 🔌 Connection
Simply connect the ReSpeaker Mic Array v2.0 to one of the USB ports on the Raspberry Pi.

## 🚀 Quick Start

1.  **Install Dependencies:**
    ```bash
    sudo apt-get install libusb-1.0-0-dev
    pip install pyusb
    ```
2.  **Add udev rule (Optional but recommended):**
    Create `/etc/udev/rules.d/99-respeaker.rules`:
    ```text
    SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666"
    ```
3.  **Run the Node:**
    ```bash
    ros2 launch xpi_sensors respeaker.launch.py
    ```
4.  **Monitor the Direction:**
    ```bash
    ros2 topic echo /respeaker/doa
    ```

## 📊 Published Topics
*   `~/doa` ([std_msgs/Int32](http://docs.ros.org/en/api/std_msgs/html/msg/Int32.html)): The angle of arrival (0-359).
*   `~/vad` ([std_msgs/Int32](http://docs.ros.org/en/api/std_msgs/html/msg/Int32.html)): Voice activity detection (0 or 1).
*   `~/is_speech` ([std_msgs/Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html)): True if speech is detected.
