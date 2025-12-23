# Non-Volatile Memory: AT24Cxxx I2C EEPROM

This block provides support for storing persistent data (robot configuration, calibration, mission state) using I2C EEPROM chips. Unlike the SD card, EEPROMs are more robust for frequent small writes and are often found on RTC modules (like DS3231) or as standalone chips.

## 🧠 Why use EEPROM?
*   **Persistence:** Store data that survives reboots (e.g., total odometer mileage, motor PID constants).
*   **Reliability:** Faster and more power-cycle resilient than writing to files on an SD card.
*   **Commonality:** Many DS3231 RTC modules already have an **AT24C32** chip onboard!

## 📦 Bill of Materials
*   Raspberry Pi
*   AT24C32 / AT24C64 / AT24C256 I2C Module
*   Jumper Wires

## 🔌 Wiring
EEPROM chips typically share the I2C bus with other sensors.

| EEPROM Pin | Raspberry Pi | Note |
|------------|--------------|-----------------------------------|
| **VCC**    | 3.3V         | Power |
| **GND**    | GND          | Ground |
| **SDA**    | GPIO 2 (SDA) | Data |
| **SCL**    | GPIO 3 (SCL) | Clock |
| **WP**     | GND          | **Write Protect**. Connect to GND to allow writing. |

**I2C Address:** Usually starts at `0x50`. If you have a DS3231 module, the RTC is at `0x68` and the EEPROM is at `0x57` or `0x50`.

## 🛠 Software Setup

1.  **Enable I2C:** `sudo raspi-config` -> Interface Options -> I2C.
2.  **Install dependencies:**
    ```bash
    pip3 install adafruit-circuitpython-eeprom
    ```

## 🚀 Usage

**Launch the node:**
```bash
ros2 run xpi_commons eeprom_node --ros-args -p eeprom_type:="AT24C32" -p size:=4096
```

### Writing Data (Example)
To write data, publish to `~/write_raw`. The first two bytes are the memory address (Big Endian).
```bash
# Write 'Hello' (72, 101, 108, 108, 111) to address 0x0010
ros2 topic pub --once /eeprom_node/write_raw std_msgs/msg/UInt8MultiArray "{data: [0, 16, 72, 101, 108, 108, 111]}"
```

## 📡 Interface

### Subscribers
*   `~/write_raw` (`std_msgs/UInt8MultiArray`): 
    *   Format: `[AddrHigh, AddrLow, Byte0, Byte1, ...]`

### Parameters
*   `i2c_address` (int, default: `0x50`): The address of the chip.
*   `size` (int, default: `32768`): Total capacity in **bytes**.
    *   **AT24C32:** 4096 bytes (32 kbits)
    *   **AT24C64:** 8192 bytes (64 kbits)
    *   **AT24C256:** 32768 bytes (256 kbits)
*   `eeprom_type` (string): For logging purposes.

## ⚠️ Important Considerations
1.  **Write Cycles:** EEPROMs have a limited lifespan (usually 1,000,000 writes). Avoid writing in a high-frequency loop (e.g., 10Hz). Only write when data actually changes.
2.  **Page Alignment:** The underlying library handles page-aligned writing, but be aware that writing large blocks takes time (~5ms per page).
3.  **Write Protect (WP):** If you cannot write to the chip, check the **WP** pin. It must be pulled to GND to enable writes. If it is High (VCC), the memory is read-only.
