# Wireless Serial: HC-12 (433MHz)

The HC-12 is a long-range (up to 1km) wireless serial communication module operating at 433MHz. It is ideal for robot-to-robot or robot-to-base station telemetry where Wi-Fi range is insufficient.

## 📌 Features
*   **Frequency:** 433.4MHz to 473.0MHz (100 channels).
*   **Range:** Up to 1000m (in open air at 5000bps).
*   **Interface:** UART (Serial).
*   **Config Mode:** Driven by the **SET** pin (Active LOW).

## 🔌 Wiring Diagram

| HC-12 Pin | Raspberry Pi Pin | Color (suggested) | Notes |
| :--- | :--- | :--- | :--- |
| VCC | 5V or 3.3V (Pin 2/4/1) | Red | 3.2V - 5.5V |
| GND | GND (Pin 6/9/14...) | Black | |
| RX | TXD (GPIO 14 / Pin 8) | Green | Connect HC-12 RX to RPi TX |
| TX | RXD (GPIO 15 / Pin 10) | White | Connect HC-12 TX to RPi RX |
| SET | GPIO 17 (Pin 11) | Orange | Pull LOW to enter AT mode |

> **Note:** For best range, use an external antenna or the included spring antenna.

## 🚀 Quick Start

1.  **Configure UART:**
    Ensure serial console is disabled in `raspi-config`.
2.  **Run the Node:**
    ```bash
    ros2 launch xpi_comms hc12.launch.py
    ```
3.  **Send Data:**
    ```bash
    ros2 topic pub /hc12/tx std_msgs/String "data: 'Hello Robot'"
    ```
4.  **Receive Data:**
    ```bash
    ros2 topic echo /hc12/rx
    ```

## ⚙️ AT Commands (via Service)
To configure the module, use the `~/set_config` service:
*   `AT+B9600`: Set baud rate to 9600.
*   `AT+C001`: Set channel to 001.
*   `AT+FU3`: Set long-range mode.
*   `AT+RX`: Get all current settings.
