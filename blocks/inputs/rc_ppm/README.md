# PPM Receiver Input (Pulse Position Modulation)

This block provides a ROS2 driver for decoding PPM (Pulse Position Modulation) signals from an RC receiver connected to a single GPIO pin. It publishes decoded channel values as `sensor_msgs/Joy` messages.

## 📦 Bill of Materials
*   Raspberry Pi
*   PPM-compatible RC Receiver
*   Jumper Wires

## 🔌 Wiring
Connect your PPM receiver's PPM output to a GPIO pin on the Raspberry Pi.

| RC Receiver PPM | Raspberry Pi | Note |
|-----------------|--------------|-----------------------------------|
| PPM Out         | GPIO 17      | Configurable via `gpio_pin` parameter (BCM numbering) |
| GND             | GND          | Common Ground                     |
| VCC             | 3.3V/5V      | Power for Receiver (check receiver spec!) |

**Important Note on GPIO Pull-up/Pull-down:**
PPM signals can be active high or active low. `gpiozero`'s `PulseInput` handles this well with `pull_up` / `pull_down` settings. By default, `gpiozero.PulseInput` uses a pull-up resistor. If your PPM signal is active low, you may need to adjust.

## 🚀 Quick Start
1.  **Launch the PPM driver**:
    ```bash
    ros2 launch xpi_inputs ppm.launch.py gpio_pin:=17
    ```
    (Adjust `gpio_pin` to your connection).

## 📡 Interface
### Publishers
*   `~/joy` (`sensor_msgs/Joy`): Publishes decoded PPM channel values.
    *   `axes`: Array of floats, normalized to `[-1.0, 1.0]`. The number of elements in the array is defined by `num_channels`.
    *   `buttons`: Not used for PPM.

### Parameters
*   `gpio_pin` (int, default: `17`): BCM GPIO pin number for PPM input.
*   `publish_rate` (float, default: `50.0`): Frequency to publish `Joy` messages.
*   `mock_hardware` (bool, default: `false`): Run in mock mode for testing.
*   `num_channels` (int, default: `8`): Number of channels expected in the PPM frame.
*   `frame_timeout_us` (int, default: `5000`): Microseconds. A pulse width greater than this indicates the end of a PPM frame.
*   `min_pulse_us` (int, default: `900`): Microseconds. Minimum expected pulse width for a channel.
*   `max_pulse_us` (int, default: `2100`): Microseconds. Maximum expected pulse width for a channel.
*   `center_pulse_us` (int, default: `1500`): Microseconds. Pulse width for the center position of a channel.

## ✅ Verification
1.  Launch the driver with your PPM receiver connected and powered on.
2.  In a new terminal, monitor the `/ppm_receiver/joy` topic:
    ```bash
    ros2 topic echo /ppm_receiver/joy
    ```
3.  Move sticks and switches on your RC transmitter; you should see the `axes` values change.

## ⚠️ Troubleshooting
*   **No data / erratic data?**
    *   Double-check GPIO wiring.
    *   Verify PPM signal characteristics (active high/low) and adjust `gpiozero.PulseInput` configuration if needed.
    *   Adjust `min_pulse_us`, `max_pulse_us`, `center_pulse_us`, `frame_timeout_us` to match your specific receiver.
*   **Permissions error?**
    *   Add your user to the `gpio` group: `sudo usermod -a -G gpio $USER`. Log out and back in.
