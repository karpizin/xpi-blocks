# GPIO Digital Input Monitor

This block provides a ROS2 driver for monitoring multiple digital input devices connected to GPIO pins (e.g., push buttons, PIR motion sensors, flame sensors, reed switches). It publishes the boolean state of each configured pin.

## 📦 Bill of Materials
*   Raspberry Pi
*   Digital input device (e.g., push button, PIR sensor module, flame sensor module, reed switch, etc.)
*   Jumper Wires
*   Resistors (if external pull-up/down is preferred over internal)

## 🔌 Wiring
Connect your digital input device(s) to the specified GPIO pins on the Raspberry Pi.

**Example: Push Button**
| Button Pin | Raspberry Pi GPIO (BCM) | Note |
|------------|-------------------------|-----------------------------------|
| One side   | GPIO 18                 | Configurable via `pins` parameter |
| Other side | GND                     | Common Ground                     |

*   If using internal `pull_up:=true`, connect button between GPIO and GND. Pressing button makes pin LOW (False).
*   If using internal `pull_down:=true`, connect button between GPIO and 3.3V. Pressing button makes pin HIGH (True).

**Example: PIR Motion Sensor**
| PIR Pin | Raspberry Pi GPIO (BCM) | Note |
|---------|-------------------------|-----------------------------------|
| VCC     | 5V                      | Power for PIR module              |
| GND     | GND                     | Common Ground                     |
| OUT     | GPIO 23                 | Output to GPIO (configurable)     |

## 🚀 Quick Start
1.  **Ensure GPIO access:** Your user needs to be in the `gpio` group.
2.  **Launch the digital input monitor:**
    ```bash
    # Example: Monitor GPIO 18 (with internal pull-up)
    ros2 launch xpi_sensors gpio_digital_input.launch.py pins:="[18]" pull_up:=true pull_down:=false
    
    # Example: Monitor multiple pins (GPIO 18, 23, 24)
    ros2 launch xpi_sensors gpio_digital_input.launch.py pins:="[18, 23, 24]" pull_up:=true pull_down:=false
    ```
    *Note: The `pins` argument must be passed as a JSON-formatted string representing a list.*

## 📡 Interface
### Publishers
*   `~/pin_<GPIO_PIN_NUMBER>/state` (`std_msgs/Bool`): Publishes `true` or `false` for each monitored pin.
    *   Example: `/digital_input_monitor/pin_18/state`
*   `~/all_pins_state` (`std_msgs/Int8MultiArray`): Publishes an array of `0`s and `1`s representing the states of all monitored pins, in the order they were provided in the `pins` parameter. Only published if multiple pins are configured.

### Parameters
*   `pins` (list of int, default: `[18, 23]`): A list of BCM GPIO pin numbers to monitor.
*   `pull_up` (bool, default: `true`): If `true`, enables the internal pull-up resistor.
*   `pull_down` (bool, default: `false`): If `true`, enables the internal pull-down resistor.
    *   *Note:* Only one of `pull_up` or `pull_down` can be `true`.
*   `publish_rate` (float, default: `10.0`): Frequency to publish pin states in Hz.
*   `mock_hardware` (bool, default: `false`): Run in mock mode without real GPIO.

## ✅ Verification
1.  Launch the driver with your digital input device(s) connected.
2.  In new terminals, monitor the individual pin topics or the combined topic:
    ```bash
    ros2 topic echo /digital_input_monitor/pin_18/state
    ros2 topic echo /digital_input_monitor/all_pins_state
    ```
3.  Trigger your sensors (e.g., press a button, move in front of a PIR sensor). You should see the corresponding `Bool` or `Int8MultiArray` values change.

## ⚠️ Troubleshooting
*   **No readings / Stuck state?**
    *   Double-check wiring.
    *   Ensure the correct `pull_up`/`pull_down` settings are used for your sensor. Many modules (like PIR) have their own pull-ups/downs and should be connected directly without extra internal resistors.
    *   Verify GPIO pin numbers are correct.
    *   Check for permissions errors (`sudo usermod -a -G gpio $USER`).
