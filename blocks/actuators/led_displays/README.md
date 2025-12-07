# MAX7219 LED Matrix 8x8 Display Driver

This block provides a ROS2 driver for controlling MAX7219-based 8x8 LED matrix displays. These displays can be cascaded to create larger display surfaces. The driver uses the `luma.led_matrix` library.

## ⚠️ Important Considerations ⚠️
*   **SPI Interface:** The MAX7219 communicates via SPI. You must enable SPI on your Raspberry Pi.
*   **Wiring:** Careful wiring is needed for cascaded displays.
*   **Power Supply:** Each 8x8 matrix can draw significant current, especially when all LEDs are on. Ensure your power supply (usually 5V) is adequate.

## 📦 Bill of Materials
*   Raspberry Pi
*   MAX7219-based 8x8 LED Matrix module(s)
*   Jumper Wires

## 🔌 Wiring
Connect the MAX7219 module(s) to the SPI pins on the Raspberry Pi. For cascaded displays, chain them together (DOUT of one to DIN of next).

| MAX7219 Pin | Raspberry Pi | Note                                      |
|-------------|--------------|-------------------------------------------|
| VCC         | 5V           | Power for the module                      |
| GND         | GND          | Common Ground                             |
| DIN         | GPIO 10 (MOSI) | Data Input (SPI MOSI)                     |
| CS          | GPIO 8 (CE0) | Chip Select (SPI CE0, configurable)       |
| CLK         | GPIO 11 (SCLK) | Clock (SPI SCLK)                          |

**Important SPI Configuration on Raspberry Pi:**
You must enable the SPI interface on your Raspberry Pi.

1.  **Enable SPI:**
    ```bash
    sudo raspi-config
    # Interface Options -> P4 SPI -> Yes
    ```
2.  **Verify:** After reboot, check for SPI devices: `ls /dev/spi*`

## 🚀 Quick Start
1.  **Install `luma.led_matrix` and `spidev`:**
    ```bash
    pip install luma.led_matrix spidev
    ```
2.  **Enable SPI** as described above.
3.  **Launch the LED matrix driver:**
    ```bash
    # Example: Single 8x8 matrix on SPI port 0, device 0
    ros2 launch xpi_actuators led_matrix.launch.py cascaded_devices:=1 gpio_port:=0 gpio_device:=0

    # Example: Two cascaded 8x8 matrices on SPI port 0, device 0
    ros2 launch xpi_actuators led_matrix.launch.py cascaded_devices:=2 gpio_port:=0 gpio_device:=0
    ```

## 📡 Interface
### Subscribers
*   `~/display_text` (`std_msgs/String`): Displays the provided text on the matrix. Text scrolls if it's longer than the display width.
*   `~/display_bitmap` (`std_msgs/Int8MultiArray`): Displays a custom bitmap.
    *   The `data` field should be a flat list of `0`s (off) and `1`s (on).
    *   The length of the array must match `device.width * device.height` (e.g., 64 for a single 8x8 matrix, 128 for two cascaded 8x8).
    *   Pixels are ordered row by row, then column by column.

### Parameters
*   `cascaded_devices` (int, default: `1`): Number of MAX7219 modules chained together.
*   `gpio_port` (int, default: `0`): SPI port number (usually `0` for CE0, `1` for CE1).
*   `gpio_device` (int, default: `0`): SPI device number (chip select).
*   `block_orientation` (int, default: `0`): Orientation of each individual 8x8 block (0, 90, -90, or 180 degrees).
*   `rotate` (int, default: `0`): Rotation of the entire cascaded display (0=0, 1=90, 2=180, 3=270).
*   `brightness` (int, default: `50`): Brightness of the display (1 to 255).
*   `mock_hardware` (bool, default: `false`): Run in mock mode.

## ✅ Verification
1.  Launch the driver with your LED matrix connected and SPI enabled.
2.  Send commands:
    *   Display text "HI":
        ```bash
        ros2 topic pub --once /led_matrix_display/display_text std_msgs/msg/String "{data: 'HI'}"
        ```
    *   Display a simple pattern (e.g., a cross on a single 8x8 matrix):
        ```bash
        # This bitmap represents a cross pattern
        ros2 topic pub --once /led_matrix_display/display_bitmap std_msgs/msg/Int8MultiArray "{data: [0,0,0,1,1,0,0,0, 0,0,0,1,1,0,0,0, 0,0,0,1,1,0,0,0, 1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1, 0,0,0,1,1,0,0,0, 0,0,0,1,1,0,0,0, 0,0,0,1,1,0,0,0]}"
        ```
3.  Observe the LED matrix displaying the text or pattern.

## ⚠️ Troubleshooting
*   **Display not working / garbage on screen?**
    *   **Wiring:** Double-check SPI connections (DIN, CS, CLK, VCC, GND).
    *   **SPI enabled:** Verify SPI is enabled in `raspi-config`.
    *   **Permissions:** `luma.led_matrix` uses `/dev/spidev0.x`, ensure user has access (usually in `spi` group, `sudo usermod -a -G spi $USER`).
    *   **`cascaded_devices`:** Ensure this parameter matches the physical number of chained matrices.
    *   **`luma.led_matrix` not found**: `pip install luma.led_matrix spidev` was successful.

---

# TM1637 7-Segment Display Driver

This block provides a ROS2 driver for controlling a TM1637-based 4-digit or 6-digit 7-segment display module. It allows displaying numbers and text, as well as adjusting brightness.

## 📦 Bill of Materials
*   Raspberry Pi
*   TM1637 4-digit or 6-digit 7-segment display module
*   Jumper Wires

## 🔌 Wiring
Connect the TM1637 module to any two available GPIO pins on the Raspberry Pi.

| TM1637 Pin | Raspberry Pi GPIO (BCM) | Note                                      |
|------------|-------------------------|-------------------------------------------|
| VCC        | 3.3V/5V                 | Power for the module (check module spec)  |
| GND        | GND                     | Common Ground                             |
| CLK        | GPIO 23                 | Configurable via `clk_pin` parameter      |
| DIO        | GPIO 24                 | Configurable via `dio_pin` parameter      |

## 🚀 Quick Start
1.  **Install `tm1637` library:**
    ```bash
    pip install tm1637
    ```
2.  **Ensure GPIO access:** Your user needs to be in the `gpio` group.
3.  **Launch the TM1637 driver:**
    ```bash
    # Example: 4-digit display on CLK=GPIO23, DIO=GPIO24
    ros2 launch xpi_actuators tm1637.launch.py clk_pin:=23 dio_pin:=24 display_digits:=4 default_text:="1234"
    ```

## 📡 Interface
### Subscribers
*   `~/display_text` (`std_msgs/String`): Displays text or numbers on the 7-segment display.
*   `~/set_brightness` (`std_msgs/UInt8`): Sets the display brightness (0-7, 7 being brightest).

### Parameters
*   `clk_pin` (int, default: `23`): BCM GPIO pin for CLK.
*   `dio_pin` (int, default: `24`): BCM GPIO pin for DIO.
*   `brightness` (int, default: `7`): Initial brightness (0-7).
*   `mock_hardware` (bool, default: `false`): Run in mock mode.
*   `display_digits` (int, default: `4`): Number of digits on your display (4 or 6).
*   `default_text` (string, default: `----`): Text to show on startup.

## ✅ Verification
1.  Launch the driver with your TM1637 display connected.
2.  Send commands:
    *   Display a number:
        ```bash
        ros2 topic pub --once /tm1637_display/display_text std_msgs/msg/String "{data: '98.7'}"
        ```
    *   Change brightness:
        ```bash
        ros2 topic pub --once /tm1637_display/set_brightness std_msgs/msg/UInt8 "{data: 2}"
        ```
3.  Observe the 7-segment display.

## ⚠️ Troubleshooting
*   **Display not working / blank?**
    *   Double-check wiring (CLK, DIO, VCC, GND).
    *   Verify GPIO access.
    *   Ensure `tm1637` library is installed (`pip install tm1637`).
    *   The `tm1637` library may sometimes require `sudo` to access GPIO directly if `gpiozero` is not used as a backend.
*   **Garbage on display?**
    *   Check `display_digits` parameter matches your physical display.
    *   Ensure the published string fits the number of digits.

