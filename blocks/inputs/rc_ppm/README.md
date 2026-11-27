# PPM Receiver Input (Pulse Position Modulation)

This block provides a ROS2 driver for decoding PPM (Pulse Position Modulation) signals from an RC receiver connected to a single GPIO pin. It publishes decoded channel values as `sensor_msgs/Joy` messages.

## 🧠 How it Works
A PPM (CPPM) stream is a train of short, fixed-width pulses. The value of each channel is the
**time between two consecutive pulse edges** (about 1000-2000 µs); a long gap (the sync gap,
several ms) marks the start of the next frame. The node uses the **pigpio** daemon, which
timestamps every edge with microsecond resolution in C, so decoding is not disturbed by Python
scheduling jitter.

> ⚠️ pigpio does **not** support the Raspberry Pi 5. Use a receiver with SBUS or CRSF output
> (`rc_sbus`, `rc_crsf` blocks) on a Pi 5.

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

**Signal level and polarity:** the Pi's GPIO is 3.3 V only; if your receiver outputs 5 V PPM, use
a level shifter or a resistor divider. Most receivers send positive pulses (`edge:=rising`); for an
inverted stream use `edge:=falling`. The internal pull-up is on by default (`pull_up`).

## 🚀 Quick Start
1.  **Install and start pigpio** (once):
    ```bash
    sudo apt-get install pigpio python3-pigpio   # or: pip install pigpio (client only)
    sudo systemctl enable --now pigpiod
    ```
2.  **Launch the PPM driver**:
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
*   `frame_timeout_us` (int, default: `3000`): Microseconds. An interval between edges at least this long is the sync gap that starts a frame.
*   `min_pulse_us` (int, default: `900`): Microseconds. Shortest valid channel interval (edge to edge).
*   `max_pulse_us` (int, default: `2100`): Microseconds. Longest valid channel interval (edge to edge).
*   `center_pulse_us` (int, default: `1500`): Microseconds. Interval that maps to `0.0`.
*   `edge` (string, default: `rising`): Edge that starts each pulse, `rising` or `falling`.
*   `pull_up` (bool, default: `true`): Enable the internal pull-up resistor.
*   `signal_timeout` (float, default: `0.25`): Seconds without a complete frame before the signal counts as lost.

### Signal loss
When no complete frame arrives for `signal_timeout` seconds (transmitter off, out of range, cable
unplugged) the node **stops publishing** and logs a warning; the command watchdogs of the motor
drivers (`cmd_timeout`) then stop the motors. A frame with a glitch (an interval outside
`min_pulse_us`..`max_pulse_us`) or the wrong number of channels is dropped as a whole.

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
    *   Is the daemon running? `pigs t` prints the current tick; the node logs an error if it cannot connect.
    *   Try `edge:=falling` for an inverted PPM stream.
    *   Check `num_channels`: frames with another channel count are dropped.
    *   Adjust `min_pulse_us`, `max_pulse_us`, `center_pulse_us`, `frame_timeout_us` to match your specific receiver.
*   **Permissions error?**
    *   GPIO access goes through `pigpiod` (runs as root), so the node itself needs no GPIO permissions.
