# WS2812 / NeoPixel Addressable RGB LED Strip Driver

This block provides a ROS2 driver for controlling WS2812B (also known as NeoPixel) addressable RGB LED strips. It uses the `rpi_ws281x` library for efficient low-level control.

## ⚠️ Important Considerations ⚠️
*   **Real-time Requirements:** `rpi_ws281x` library requires precise timing provided by the Raspberry Pi's PWM or PCM peripheral. It often needs to be run with root privileges or specific user permissions.
*   **GPIO Conflicts:** The library uses specific GPIO pins for PWM/PCM. Ensure no other HATs or processes are using these pins.
*   **Power Supply:** WS2812 LEDs can draw a significant amount of current.
    *   **NEVER** power a large strip directly from the Raspberry Pi's 5V pin.
    *   Use a separate, appropriately sized 5V power supply.
    *   Ensure common ground between the Pi and the LED power supply.
*   **Logic Level Shifting:** While many WS2812 strips work with 3.3V data from the Pi, some may require a 5V logic level shifter for reliable operation, especially over longer distances.

## 📦 Bill of Materials
*   Raspberry Pi
*   WS2812B/NeoPixel LED Strip
*   Separate 5V Power Supply (for LEDs)
*   Jumper Wires
*   **Optional:** 300-500 Ohm Resistor (between Pi GPIO and LED data-in, for protection)
*   **Optional:** 1000µF Capacitor (across LED power supply, for stability)
*   **Optional:** Logic Level Shifter (e.g., 74AHCT125 or 74HCT245)

## 🔌 Wiring
Connect the data input of your WS2812 strip to a suitable GPIO pin on the Raspberry Pi.

| WS2812 Strip Pin | Raspberry Pi GPIO (BCM) | Note                                      |
|------------------|-------------------------|-------------------------------------------|
| VCC (5V)         | **Separate 5V PSU +**   | **DO NOT power from Pi's 5V for large strips!** |
| GND              | **Separate 5V PSU - & RPi GND** | **Common ground is essential!** |
| DIN (Data In)    | GPIO 18 (PWM0)          | Best choice. Configurable via `led_pin`.  |
|                  | (e.g. GPIO 10 - PCM)    | Alternative, configurable via `led_pin`.  |

## 🚀 Quick Start
1.  **Install `rpi_ws281x`:**
    ```bash
    pip install rpi_ws281x adafruit-circuitpython-neopixel # neopixel is an easier wrapper, rpi_ws281x is the core
    ```
2.  **Enable GPIO Access / Permissions:**
    *   The `rpi_ws281x` library typically requires root privileges to access the DMA controller for precise timing.
    *   You might need to run the ROS2 node with `sudo` or configure `udev` rules.
    *   **Recommendation:** For development, `sudo ros2 launch ...` might work. For deployment, consider `systemd` services or `udev` rules.
3.  **Launch the WS2812 driver:**
    ```bash
    ros2 launch xpi_actuators ws2812.launch.py led_count:=30 led_pin:=18
    ```

## 📡 Interface
### Subscribers
*   `~/pixel_<ID>/set_color` (`std_msgs/ColorRGBA`): Sets the color of a specific pixel (ID from 0 to `led_count-1`).
    *   `r, g, b` values (0.0 to 1.0) for Red, Green, Blue.
    *   `a` (alpha) value (0.0 to 1.0) is used for White component for RGBW strips (or acts as brightness).
*   `~/strip/set_color` (`std_msgs/ColorRGBA`): Sets all pixels on the strip to the given color.

### Parameters
*   `led_count` (int, default: `30`): Number of LEDs in your strip.
*   `led_pin` (int, default: `18`): BCM GPIO pin. Recommended: `18` (PWM0) or `10` (PCM).
*   `led_freq_hz` (int, default: `800000`): Signal frequency (800kHz is standard).
*   `led_dma` (int, default: `10`): DMA channel.
*   `led_brightness` (int, default: `255`): Global brightness (0-255).
*   `led_invert` (bool, default: `false`): Invert signal.
*   `led_channel` (int, default: `0`): PWM channel (0 or 1).
*   `mock_hardware` (bool, default: `false`): Run in mock mode.
*   `update_rate` (float, default: `30.0`): Internal update rate for `strip.show()`.

## ✅ Verification
1.  Launch the driver with your WS2812 strip connected and powered.
2.  Send commands:
    *   Set pixel 0 to red:
        ```bash
        ros2 topic pub --once /ws2812_strip_driver/pixel_0/set_color std_msgs/msg/ColorRGBA "{r: 1.0, g: 0.0, b: 0.0, a: 0.0}"
        ```
    *   Set the whole strip to blue:
        ```bash
        ros2 topic pub --once /ws2812_strip_driver/strip/set_color std_msgs/msg/ColorRGBA "{r: 0.0, g: 0.0, b: 1.0, a: 0.0}"
        ```
    You should see the LEDs change color.

## ⚠️ Troubleshooting
*   **LEDs not lighting up / erratic behavior?**
    *   **POWER!** Ensure the LEDs have their own robust 5V power supply and a common ground with the Pi.
    *   **Permissions:** Try running the launch command with `sudo`. If it works with `sudo`, it's a permissions issue.
    *   **Wiring:** Double-check data pin (DIN) connection to the correct GPIO.
    *   **Resistor/Level Shifter:** Try adding the optional resistor or a 5V logic level shifter.
    *   **DMA conflict:** If other hardware is using DMA, try a different `led_dma` channel or `led_pin`.
*   **"rpi_ws281x not found"**: Ensure `pip install rpi_ws281x` was successful.
