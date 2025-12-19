# LCD 1602 (I2C): Character Display

The **LCD 1602** is a basic character display capable of showing 16 characters across 2 lines. It is widely used for status output (IP address, Battery %, Mode).

This block supports the **I2C version** (equipped with a PCF8574 "backpack").

## ⚡ Wiring (I2C)

| LCD Pin | Raspberry Pi Pin | Description          |
| :---    | :---             | :---                 |
| **VCC** | 5V (Pin 2 or 4)  | Power (Requires 5V!) |
| **GND** | GND (Pin 6)      | Ground               |
| **SDA** | GPIO 2 (Pin 3)   | I2C Data             |
| **SCL** | GPIO 3 (Pin 5)   | I2C Clock            |

> **⚠️ Note:** While the LCD requires 5V power, the I2C logic on the backpack is usually tolerant enough to work with the Pi's 3.3V logic SDA/SCL without a level shifter. If you have issues, use a logic level shifter.

## 📦 Bill of Materials
*   1x LCD 1602 Module with I2C Backpack
*   4x Jumper Wires

## 🚀 Usage

### 1. Find Address
The I2C address is usually **0x27** or **0x3F**. Check it:
```bash
i2cdetect -y 1
```

### 2. Launch the Node
```bash
ros2 launch xpi_actuators lcd1602.launch.py
```

### 3. Send Text
**Write to Line 1:**
```bash
ros2 topic pub /lcd1602/line1 std_msgs/msg/String "data: 'Hello World!'" -1
```

**Write to Line 2:**
```bash
ros2 topic pub /lcd1602/line2 std_msgs/msg/String "data: 'ROS2 on RPi'" -1
```

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `i2c_bus` | int | `1` | I2C bus number. |
| `i2c_address` | int | `0x27` | Sensor I2C address (0x27 or 0x3F). |
| `cols` | int | `16` | Number of columns. |
| `rows` | int | `2` | Number of rows. |

## 🧩 Topics Interface

### Subscribers
*   `~/line1` (`std_msgs/msg/String`) - Updates text on the first row.
*   `~/line2` (`std_msgs/msg/String`) - Updates text on the second row.
*   `~/write` (`std_msgs/msg/String`) - Writes text sequentially (wraps automatically).
*   `~/clear` (`std_msgs/msg/Empty`) - Clears the display.
