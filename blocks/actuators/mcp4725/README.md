# MCP4725 12-bit Digital-to-Analog Converter (DAC)

The MCP4725 is a 12-bit DAC that allows you to generate a true analog voltage from your Raspberry Pi's I2C bus. Unlike PWM, which is a digital signal pulsed rapidly, the DAC provides a steady, continuous voltage level.

## 🧠 Features
*   **Resolution:** 12-bit (4096 steps).
*   **Interface:** I2C.
*   **EEPROM:** Built-in memory allows storing a default power-up voltage.

## 📦 Bill of Materials
*   Raspberry Pi (4, 5, or Zero)
*   MCP4725 DAC Breakout Module
*   Jumper Wires

## 🔌 Wiring

| MCP4725 Pin | Raspberry Pi | Note |
|-------------|--------------|---------------------------|
| VCC         | 3.3V (or 5V) | Power and Reference |
| GND         | GND          | |
| SDA         | SDA (Pin 3)  | I2C Data |
| SCL         | SCL (Pin 5)  | I2C Clock |
| OUT         | -            | **Analog Voltage Output** |

## 🛠 Software Setup

1.  **Enable I2C:** `sudo raspi-config` -> Interface Options -> I2C.
2.  **Dependencies:**
    ```bash
    pip install adafruit-circuitpython-mcp4725
    ```

## 🚀 Usage

**Launch the node:**
```bash
ros2 run xpi_actuators mcp4725_node
```

**Set output to 50% voltage:**
```bash
ros2 topic pub /mcp4725_node/cmd_normalized std_msgs/msg/Float32 "{data: 0.5}" --once
```

**Set output to specific voltage (e.g., 1.5V):**
```bash
ros2 topic pub /mcp4725_node/cmd_voltage std_msgs/msg/Float32 "{data: 1.5}" --once
```

## 📡 Interface

### Subscribers
*   `~/cmd_normalized` (`std_msgs/Float32`): Value from 0.0 to 1.0.
*   `~/cmd_voltage` (`std_msgs/Float32`): Absolute voltage value (up to VCC).

### Parameters
*   `i2c_address` (int, default: `0x62`): I2C address of the DAC.
*   `vcc` (float, default: `3.3`): Supply voltage, used for voltage-to-raw calculations.
