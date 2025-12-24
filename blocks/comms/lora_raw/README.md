# Communication: Raw LoRa (SX1276/SX1278)

This block provides low-level access to the SX1276/SX1278 LoRa transceivers via SPI. It is designed for point-to-point communication with minimum latency and maximum control over radio parameters.

## 📌 Features
*   **Long Range:** Up to several kilometers depending on terrain and antenna.
*   **Custom Protocols:** Send raw byte buffers or strings.
*   **Adjustable Parameters:** Frequency, Spreading Factor (SF), Bandwidth (BW), and Coding Rate (CR).
*   **Interface:** SPI.

## 🔌 Wiring Diagram

| SX127x Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VCC | 3.3V (Pin 1) | Red | **Do NOT use 5V!** |
| GND | GND (Pin 9) | Black | |
| SCK | SCLK (GPIO 11 / Pin 23) | Yellow | |
| MOSI | MOSI (GPIO 10 / Pin 19) | Blue | |
| MISO | MISO (GPIO 9 / Pin 21) | Green | |
| NSS (CS) | CE1 (GPIO 7 / Pin 26) | White | Or CE0 |
| DIO0 | GPIO 25 (Pin 22) | Orange | Interrupt Pin |
| RESET | GPIO 17 (Pin 11) | Grey | |

> **⚠️ WARNING:** Always connect an antenna before powering the module, otherwise the transmitter may be damaged.

## 🚀 Quick Start

1.  **Enable SPI:** Use `raspi-config`.
2.  **Install Dependencies:**
    ```bash
    pip install pyLoRa
    ```
3.  **Run the Node:**
    ```bash
    ros2 launch xpi_comms lora_raw.launch.py
    ```
4.  **Transmit Data:**
    ```bash
    ros2 topic pub /lora/tx std_msgs/msg/String "{data: 'Ping'}"
    ```

## 📊 Published Topics
*   `~/rx` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)): Received data packets.
*   `~/rssi` ([std_msgs/Int32](http://docs.ros.org/en/api/std_msgs/html/msg/Int32.html)): Received Signal Strength Indicator (dBm).
