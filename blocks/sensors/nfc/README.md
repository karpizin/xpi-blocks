# RFID & NFC Identification

This block provides support for identifying objects or users using Radio Frequency Identification (RFID) and Near Field Communication (NFC) technologies.

## 🧠 Comparison: PN532 vs. RC522

| Feature | PN532 (Recommended) | RC522 |
| :--- | :--- | :--- |
| **Technology** | Full NFC (Read/Write/P2P/Emulation) | RFID (Read/Write) |
| **Interfaces** | I2C, SPI, UART (Switchable) | SPI (Primary), I2C, UART |
| **Frequency** | 13.56 MHz | 13.56 MHz |
| **Supported Tags** | Mifare, NTAG, FeliCa, ISO14443 | Mifare (Classic/Ultralight) |
| **Voltage** | 3.3V - 5V | 3.3V (Logic is NOT 5V tolerant!) |

---

## 🛠 PN532 (I2C Mode) - Implemented

The **PN532** is the most powerful and versatile NFC chip available for hobbyists. It is used by the `nfc_reader_node`.

### 📦 Bill of Materials
*   Raspberry Pi
*   PN532 Module (Red "V3" boards are common)
*   Jumper Wires

### 🔌 Wiring (I2C)
**Set switches for I2C:** Check your module's DIP switches. For Red boards, typically `SET 0=ON, 1=OFF`.

| PN532 Pin | Raspberry Pi | Note |
|-----------|--------------|-----------------------------------|
| **VCC**   | 5V           | Power (Pin 2 or 4) |
| **GND**   | GND          | Ground (Pin 6) |
| **SDA**   | GPIO 2 (SDA) | Data (Pin 3) |
| **SCL**   | GPIO 3 (SCL) | Clock (Pin 5) |

**Important:** Many PN532 modules have internal 10k pull-ups to 5V. If you experience I2C issues or want to be safe, use a logic level shifter or remove the onboard pull-ups.

### 🚀 Usage
```bash
ros2 launch xpi_sensors nfc.launch.py
```

---

## 🛠 MFRC522 (SPI Mode) - Coming Soon

The **RC522** is a budget-friendly alternative primarily for 13.56 MHz RFID tags (Mifare).

### 🔌 Wiring (SPI)
RC522 requires more pins and is strictly **3.3V**.

| RC522 Pin | Raspberry Pi | Note |
|-----------|--------------|-----------------------------------|
| **VCC**   | 3.3V         | **DO NOT CONNECT TO 5V!** |
| **RST**   | GPIO 25      | Reset (Pin 22) |
| **GND**   | GND          | Ground (Pin 6) |
| **MISO**  | GPIO 9       | SPI MISO (Pin 21) |
| **MOSI**  | GPIO 10      | SPI MOSI (Pin 19) |
| **SCK**   | GPIO 11      | SPI Clock (Pin 23) |
| **SDA (SS)**| GPIO 8     | SPI CE0 (Pin 24) |

### 🛠 Software Setup
Requires `spidev` and `mfrc522` python library.

---

## 📡 ROS2 Interface (`nfc_reader_node`)

### Publishers
*   `~/tag_uid` (`std_msgs/String`): The unique ID of the tag in Hex (e.g., `DE AD BE EF`).
*   `~/tag_detected` (`std_msgs/Bool`): Stays `True` as long as the tag is within range.

### Parameters
*   `polling_rate` (float, default: `5.0`): How often to scan for tags.
*   `i2c_address` (int, default: `0x24`): The I2C address of the PN532.

## 💡 Practical Use Cases
1.  **Charging Station:** Place an NFC tag on the dock. The robot identifies the dock UID to confirm it has reached the correct charging point.
2.  **User Access:** Use Mifare keyfobs to unlock specific robot functions or "wake up" the robot.
3.  **Navigation Hints:** Place tags on the floor at key locations (intersections, rooms) to provide a "ground truth" coordinate correction.

## ⚠️ Troubleshooting
*   **"Found PN532" but no tags read:** Ensure the tag is 13.56 MHz. 125 KHz (EM4100) tags will NOT work.
*   **I2C Errors:** Check wiring and ensure I2C is enabled in `raspi-config`. Ensure the DIP switches are correctly set for I2C mode.