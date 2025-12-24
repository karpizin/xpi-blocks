# Analog Expander: CD74HC4067 (16-Channel Mux)

The CD74HC4067 is a high-speed CMOS 16-channel analog multiplexer/demultiplexer. It acts like a 16-way rotary switch, allowing you to connect 16 analog sensors to a single Analog-to-Digital Converter (ADC) channel.

## 📌 Features
*   **16-to-1 Expansion:** Read 16 sensors using 4 selection pins + 1 ADC channel.
*   **Bidirectional:** Can be used for both inputs (sensors) and outputs.
*   **Low "ON" Resistance:** Minimal signal degradation.

## 🔌 Wiring Diagram: The "Analog Sandwich"

To use this with a Raspberry Pi, you need an ADC (like **ADS1115**) because the Pi does not have native analog inputs.

### 1. Control Pins (RPi to Mux)
| CD74HC4067 Pin | Raspberry Pi Pin | Color | Note |
| :--- | :--- | :--- | :--- |
| VCC | 3.3V (Pin 1) | Red | |
| GND | GND (Pin 9) | Black | |
| S0 | GPIO 17 (Pin 11) | Blue | Binary bit 0 |
| S1 | GPIO 27 (Pin 13) | Yellow | Binary bit 1 |
| S2 | GPIO 22 (Pin 15) | Green | Binary bit 2 |
| S3 | GPIO 10 (Pin 19) | White | Binary bit 3 |
| EN | GND | - | Enable (Active LOW) |

### 2. Signal Path (Mux to ADC)
| CD74HC4067 Pin | ADS1115 Pin | Note |
| :--- | :--- | :--- |
| **SIG (Common)** | **A0** | The output of the mux goes to ADC input |

---

## 🚀 Quick Start

1.  **Launch the Mux Node:**
    ```bash
    ros2 launch xpi_sensors analog_mux_4067.launch.py
    ```
2.  **Launch the ADC Node:**
    ```bash
    ros2 launch xpi_sensors ads1115.launch.py
    ```
3.  **Switch to Channel 5:**
    ```bash
    ros2 service call /analog_mux/select_channel std_srvs/srv/SetBool "{data: true}" # (See usage below)
    ```
    *Note: We use a custom topic or service to select the channel.*

## 📐 Implementation Logic
The node sets the GPIO pins S0-S3 based on the selected channel (0-15):
*   Channel 0: `0000` (All pins LOW)
*   Channel 5: `0101` (S0=H, S1=L, S2=H, S3=L)
*   Channel 15: `1111` (All pins HIGH)

---

## 📊 Published Topics & Services
*   **`~/select_channel`** (Service): Select which analog input (0-15) to route to the ADC.
*   **`~/current_channel`** (Topic): Publishes the currently active channel.
