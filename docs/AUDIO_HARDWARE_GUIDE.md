# XPI-Blocks: Audio Hardware Guide

This document covers supported hardware for sound capture (microphones, arrays) and output (amplifiers, speakers) on Raspberry Pi / XPI platforms.

## 🎤 Sound Capture (Microphones & Arrays)

### 1. Multi-Microphone Arrays (Recommended for SSL/DOA)
Best for: Direction of Arrival (DOA), Beamforming, and Voice Interaction.

| Model | Interface | Mic Count | Pros |
| :--- | :--- | :--- | :--- |
| **ReSpeaker Mic Array v2.0** | USB | 4 | **Recommended.** Onboard DSP for DOA and AEC. Plug & Play. |
| **ReSpeaker 4-Mic HAT** | GPIO/I2S | 4 | Compact, sits on top of RPi. |
| **ReSpeaker 2-Mics HAT** | GPIO/I2S | 2 | Very low cost, basic voice interaction. |
| **PlayStation Eye** | USB | 4 | Extremely cheap legacy option (4-mic array). |

### 2. Digital I2S Microphones (MEMS)
Best for: Compact robots and custom integration.

| Model | Interface | Type | Notes |
| :--- | :--- | :--- | :--- |
| **INMP441** | I2S | Digital | High performance, low noise. Requires 3.3V. |
| **SPH0645LM4H** | I2S | Digital | Great sensitivity. Works well with `xpi_sensors/audio_level_node`. |

### 3. Generic USB Microphones
Best for: Simple noise level monitoring and basic AI analysis.
*   **Any standard USB Audio Class (UAC) microphone** will work via ALSA.
*   **USB Sound Cards (Dongles):** Use these to connect standard 3.5mm analog microphones.

---

## 🔊 Sound Output (Amplifiers & Speakers)

If your robot needs to speak (TTS) or play alerts, you need an output stage.

### 1. I2S Amplifiers (Direct Digital)
*   **MAX98357A:** Mono 3W Class D amplifier. Connects directly to RPi I2S pins. No external DAC needed.
*   **Adafruit I2S 3W Stereo Speaker Bonnet:** Dual MAX98357A for stereo sound.

### 2. Analog/USB Amplifiers
*   **PAM8403:** Tiny 2-channel 3W amplifier. Use with a USB sound card or RPi audio jack.
*   **USB Powered Speakers:** The simplest way—plug into USB for power and 3.5mm for audio.

---

## 🛠 Wiring Examples (I2S)

### INMP441 Microphone to RPi
| INMP441 Pin | RPi Pin | Note |
| :--- | :--- | :--- |
| VCC | 3.3V (Pin 1) | |
| GND | GND (Pin 9) | |
| L/R | GND | Set to Left channel |
| WS | GPIO 19 (Pin 35) | Word Select |
| SCK | GPIO 18 (Pin 12) | Clock |
| SD | GPIO 20 (Pin 38) | Serial Data |

### MAX98357A Amplifier to RPi
| MAX98357A Pin | RPi Pin | Note |
| :--- | :--- | :--- |
| Vin | 5V (Pin 2) | |
| GND | GND (Pin 6) | |
| LRC | GPIO 19 (Pin 35) | |
| BCLK | GPIO 18 (Pin 12) | |
| DIN | GPIO 21 (Pin 40) | |
