# W25Qxx SPI Flash Memory

The W25Qxx series (W25Q16, W25Q32, W25Q64, W25Q128) are popular SPI Flash memory chips providing from 2MB to 16MB of non-volatile storage. They are faster than EEPROMs and provide higher capacity, but require sector erasing before writing.

## 📦 Bill of Materials
*   Raspberry Pi (4, 5, or Zero)
*   W25Qxx Module (e.g., W25Q64 or W25Q128)
*   Jumper Wires

## 🔌 Wiring

The module uses the SPI interface.

| W25Qxx Pin | Raspberry Pi Pin | Note |
|------------|------------------|---------------------------|
| VCC        | 3.3V (Pin 1)     | |
| GND        | GND (Pin 6)      | |
| CS         | GPIO 8 (Pin 24)  | SPI0 CE0 |
| CLK        | GPIO 11 (Pin 23) | SPI0 SCLK |
| DO (MISO)  | GPIO 9 (Pin 21)  | SPI0 MISO |
| DI (MOSI)  | GPIO 10 (Pin 19) | SPI0 MOSI |

## 🛠 Software Setup

1.  **Enable SPI:**
    ```bash
    sudo raspi-config
    ```
    Navigate to `Interface Options` -> `SPI` and select `Yes`.

2.  **Dependencies:**
    The node requires `spidev`. It is usually pre-installed on Raspberry Pi OS, but can be added via:
    ```bash
    pip install spidev
    ```

## 🚀 Usage

**Launch the node:**
```bash
ros2 run xpi_commons w25qxx_node
```

**Parameters:**
*   `spi_bus` (int, default: `0`): SPI bus number.
*   `spi_device` (int, default: `0`): SPI chip select index (CE0/CE1).
*   `spi_speed` (int, default: `10000000`): Clock speed in Hz (10MHz).
*   `mock_hardware` (bool, default: `false`): Run in simulation mode.

## 📡 Interface

### Subscribers
*   `~/write_raw` (`std_msgs/UInt8MultiArray`): Write data to Flash.
    *   Format: `[AddrHigh, AddrMid, AddrLow, Data0, Data1, ...]`
    *   **Warning:** You must ensure the sector is erased before writing. (Sector erase service coming soon).

### Auto-Detection
On startup, the node reads the JEDEC ID from the chip and identifies its capacity (e.g., "Detected Flash: W25Q64 (8MB)").

## ⚠️ Important Considerations
*   **Erase before Write:** Unlike RAM or EEPROM, Flash bits can only be changed from 1 to 0 during programming. To set a bit back to 1, you must erase the entire sector (typically 4KB).
*   **Write Cycles:** These chips have a limit of approximately 100,000 erase/write cycles. Avoid frequent continuous writing in tight loops.
