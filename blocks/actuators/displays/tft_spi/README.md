# SPI TFT Displays: ST7789 / ST7735

These are high-quality, colorful IPS and TFT displays. They are ideal for showing robot "faces", camera streams, or detailed system telemetry.

## 📦 Bill of Materials
*   Raspberry Pi
*   ST7789 (e.g. 240x240 IPS) or ST7735 (e.g. 128x160 TFT) Module
*   Jumper Wires

## 🔌 Wiring (SPI)
These displays use the SPI bus.

| Display Pin | Raspberry Pi | Note |
|-------------|--------------|-----------------------------------|
| **VCC**     | 3.3V or 5V   | Check your module specs. |
| **GND**     | GND          | Ground |
| **SCL (CLK)**| GPIO 11     | SPI Clock (Pin 23) |
| **SDA (MOSI)**| GPIO 10    | SPI MOSI (Pin 19) |
| **RES (RST)**| GPIO 27     | Reset (Pin 13) |
| **DC**      | GPIO 25      | Data/Command (Pin 22) |
| **CS**      | GPIO 8       | SPI CE0 (Pin 24) |
| **BLK (BL)**| GPIO 24      | Backlight (Pin 18) |

---

## 🛠 Software Setup

1.  **Enable SPI:** `sudo raspi-config` -> Interface Options -> SPI -> Yes.
2.  **Install Dependencies:**
    ```bash
    pip3 install adafruit-circuitpython-st7789 adafruit-circuitpython-st7735 adafruit-circuitpython-rgb-display
    ```

## 🚀 Usage

**Launch for ST7789 (240x240):**
```bash
ros2 launch xpi_actuators tft_display.launch.py display_type:=ST7789 width:=240 height:=240
```

### Displaying an Image
You can send any ROS2 image message to the display:
```bash
# Example: If you have a camera node running
ros2 topic next /camera/image_raw /tft_display/image_in
```

### Displaying Text
The node supports quick text rendering:
```bash
ros2 topic pub /tft_display/text_in std_msgs/msg/String "{data: 'Battery: 12.4V\nStatus: Active'}"
```

## 📡 Interface

### Subscribers
*   `~/image_in` (`sensor_msgs/Image`): Full-screen graphics.
*   `~/text_in` (`std_msgs/String`): Simple white-on-black text output.

### Parameters
*   `display_type`: `ST7789` or `ST7735`.
*   `width`/`height`: Pixels.
*   `rotation`: `0`, `90`, `180`, `270`.
*   `baudrate`: SPI speed (default 24MHz).

## ⚠️ Performance Note
The display update is done via Python and `PIL`. For a 240x240 display, expect around **10-15 FPS**. This is sufficient for status screens and "faces", but not for high-speed video.
