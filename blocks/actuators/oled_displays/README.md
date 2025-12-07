# SSD1306 / SH1106 I2C OLED Display Driver

This block provides a ROS2 driver for controlling SSD1306 or SH1106 based I2C OLED displays. It allows displaying text and custom bitmaps, supporting various resolutions.

## ⚠️ Important Considerations ⚠️
*   **I2C Interface:** You must enable I2C on your Raspberry Pi.
*   **Resolution:** OLED displays come in different resolutions (e.g., 128x64, 128x32, 96x16, 64x48). Ensure the `width` and `height` parameters match your display.
*   **Libraries:** This driver relies on the `luma.oled` library, which internally uses `Pillow` for image manipulation.

## 📦 Bill of Materials
*   Raspberry Pi
*   SSD1306 or SH1106 OLED Display module (I2C interface)
*   Jumper Wires

## 🔌 Wiring
Connect the OLED module to the I2C pins on the Raspberry Pi.

| OLED Pin | Raspberry Pi | Note                                      |
|----------|--------------|-------------------------------------------|
| VCC      | 3.3V         | Power for the module (Pin 1)              |
| GND      | GND          | Common Ground (Pin 6)                     |
| SDA      | GPIO 2 (SDA) | Data line (Pin 3)                         |
| SCL      | GPIO 3 (SCL) | Clock line (Pin 5)                        |

**Important I2C Configuration on Raspberry Pi:**
You must enable the I2C interface on your Raspberry Pi.

1.  **Enable I2C:**
    ```bash
    sudo raspi-config
    # Interface Options -> P3 I2C -> Yes
    ```
2.  **Verify I2C Address:** Check the OLED's I2C address (usually 0x3C or 0x3D)
    ```bash
    sudo apt install i2c-tools
    i2cdetect -y 1 # Or 0 if using an older Pi
    # Look for '3c' or '3d' in the output.
    ```

## 🚀 Quick Start
1.  **Install `luma.oled` and `Pillow`:**
    ```bash
    pip install luma.oled Pillow
    ```
2.  **Enable I2C** as described above.
3.  **Launch the OLED driver:**
    ```bash
    # Example: 128x64 OLED on I2C address 0x3C
    ros2 launch xpi_actuators ssd1306.launch.py i2c_address:=0x3C width:=128 height:=64

    # Example: 128x32 OLED
    ros2 launch xpi_actuators ssd1306.launch.py i2c_address:=0x3C width:=128 height:=32 default_text:="Small OLED"
    ```

## 📡 Interface
### Subscribers
*   `~/display_text` (`std_msgs/String`): Displays the provided text on the OLED. The text will be wrapped or truncated if too long.
*   `~/display_bitmap` (`std_msgs/Int8MultiArray`): Displays a custom 1-bit (black/white) bitmap.
    *   The `data` field should be a flat list of `0`s (off) and `1`s (on).
    *   The length of the array must match `width * height`.
    *   Pixels are ordered row by row.

### Parameters
*   `i2c_bus` (int, default: `1`): I2C bus number.
*   `i2c_address` (int, default: `0x3C`): I2C address of the OLED.
*   `width` (int, default: `128`): Display width in pixels.
*   `height` (int, default: `64`): Display height in pixels.
*   `rotation` (int, default: `0`): Display rotation in degrees (0, 90, 180, 270).
*   `font_size` (int, default: `10`): Default font size for text display.
*   `default_text` (string, default: `Hello XPI!`): Text to display on startup.
*   `mock_hardware` (bool, default: `false`): Run in mock mode.

## ✅ Verification
1.  Launch the driver with your OLED connected and I2C enabled.
2.  Send commands:
    *   Display "Hello World!":
        ```bash
        ros2 topic pub --once /oled_display/display_text std_msgs/msg/String "{data: 'Hello World!'}"
        ```
    *   Display a simple pattern (e.g., a square on a 128x64 OLED):
        ```bash
        # This bitmap represents a simple square pattern in the middle
        # It needs to be 128*64 = 8192 elements long
        # ... (imagine 8192 numbers here for demonstration) ...
        # For a 10x10 square in the center of 128x64:
        # data = [0]*(64*128)
        # for y in range(27, 37): # 10 rows
        #   for x in range(59, 69): # 10 cols
        #       data[y*128 + x] = 1
        # ros2 topic pub --once /oled_display/display_bitmap std_msgs/msg/Int8MultiArray "{data: [ ... huge array ... ]}"
        ```
3.  Observe the OLED display.

## ⚠️ Troubleshooting
*   **Display not working / blank screen?**
    *   **Wiring:** Double-check I2C connections (SDA/SCL, VCC/GND).
    *   **I2C enabled:** Verify I2C is enabled in `raspi-config`.
    *   **Permissions:** `sudo usermod -a -G i2c $USER`.
    *   **`i2c_address`:** Ensure this parameter matches your display's address.
    *   **`width` / `height`:** Incorrect resolution can cause blank or corrupted display.
    *   **"luma.oled not found"**: `pip install luma.oled Pillow` was successful.
