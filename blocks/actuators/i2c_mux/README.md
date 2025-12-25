# TCA9548A I2C Multiplexer (8-Channel)

The TCA9548A is an essential tool for robotics projects that use multiple I2C sensors with the same address (e.g., eight SSD1306 OLEDs or several VL53L1X distance sensors). It allows you to switch between 8 downstream I2C buses using a single command.

## 🧠 How it Works
The multiplexer acts as a gatekeeper. By sending a control byte to the TCA9548A, you choose which of the 8 channels is connected to the main Raspberry Pi I2C bus. Only one channel can be active at a time (or all can be disabled).

## 📦 Bill of Materials
*   Raspberry Pi (4, 5, or Zero)
*   TCA9548A Breakout Board
*   Multiple I2C devices (with conflicting addresses)
*   Jumper Wires

## 🔌 Wiring

| TCA9548A Pin | Raspberry Pi | Note |
|--------------|--------------|---------------------------|
| VIN          | 3.3V (Pin 1) | |
| GND          | GND (Pin 6)  | |
| SDA          | SDA (Pin 3)  | Main I2C Data |
| SCL          | SCL (Pin 5)  | Main I2C Clock |
| RST          | -            | Optional Reset pin |

**Downstream:** Connect your sensors to SD0/SC0 through SD7/SC7.

## 🛠 Software Setup

1.  **Enable I2C:** `sudo raspi-config` -> Interface Options -> I2C.
2.  **Verify device:**
    ```bash
    i2cdetect -y 1
    ```
    You should see the address `0x70` (default).

## 🚀 Usage

**Launch the node:**
```bash
ros2 run xpi_commons i2c_mux_node
```

**Switch channel via command line:**
```bash
# Connect to channel 3
ros2 topic pub /i2c_mux_node/cmd std_msgs/msg/Int32 "{data: 3}" --once

# Disable all channels
ros2 topic pub /i2c_mux_node/cmd std_msgs/msg/Int32 "{data: -1}" --once
```

**Switch channel via Service:**
```bash
ros2 service call /i2c_mux_node/set_channel example_interfaces/srv/SetInt32 "{data: 5}"
```

## 📡 Interface

### Subscribers
*   `~/cmd` (`std_msgs/Int32`): Switch to channel 0-7. Use -1 to disconnect all.

### Services
*   `~/set_channel` (`example_interfaces/srv/SetInt32`): Change channel and get confirmation.

## ⚠️ Important Note
After switching the channel via the multiplexer, you may need to wait a few milliseconds before communicating with the sensor on that channel to allow the bus to stabilize.
