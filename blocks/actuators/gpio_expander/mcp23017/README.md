# GPIO Expander: MCP23017 (16-Channel I2C)

The MCP23017 is a 16-bit, general-purpose parallel I/O expansion for I2C bus. It provides 16 additional GPIO pins for the Raspberry Pi, divided into two 8-bit ports (PORT A and PORT B).

## 📌 Features
*   **16 Channels:** Each pin can be configured independently as an input or output.
*   **Interrupt Support:** Can notify the Pi when a state changes.
*   **Internal Pull-ups:** 100k Ohm pull-up resistors can be enabled via software.
*   **Interface:** I2C (Address 0x20 to 0x27, configurable via A0-A2 pins).

## 🔌 Wiring Diagram

| MCP23017 Pin | Raspberry Pi Pin | Color (suggested) | Note |
| :--- | :--- | :--- | :--- |
| VCC (9) | 3.3V (Pin 1) | Red | |
| GND (10) | GND (Pin 9) | Black | |
| SCL (12) | SCL (GPIO 3) | Yellow | |
| SDA (13) | SDA (GPIO 2) | Blue | |
| RESET (18) | VCC | - | Pull HIGH for normal operation |
| A0, A1, A2 | GND | - | Sets I2C address to 0x20 |

---

## 🚀 Quick Start

1.  **Enable I2C:** Use `raspi-config` to enable the I2C interface.
2.  **Install Dependencies:**
    ```bash
    pip install adafruit-circuitpython-mcp23017
    ```
3.  **Run the Node:**
    ```bash
    ros2 launch xpi_actuators mcp23017.launch.py
    ```
4.  **Set Output (Pin 0):**
    ```bash
    ros2 topic pub --once /mcp23017/set_output std_msgs/msg/String "data: '0:1'" # Pin 0 HIGH
    ```

## 📊 Published Topics
*   `~/inputs` ([std_msgs/Int32](http://docs.ros.org/en/api/std_msgs/html/msg/Int32.html)): Current state of all 16 pins as a bitmask.

## 📡 Subscribed Topics
*   `~/set_output` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)): Set pin state using "pin:state" format (e.g. "7:1").
