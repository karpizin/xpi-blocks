# IP5306: Power Bank Management IC (I2C)

The **IP5306** is a popular power bank management chip often used in budget Raspberry Pi Power HATs, T-Power modules, and M5Stack devices. It manages charging, discharging, and provides basic battery level telemetry over I2C.

## ⚡ Features
*   **Battery Percentage:** Fuel gauge monitoring (0-100%).
*   **Charging Status:** Detects if the device is plugged into USB power.
*   **Single-Chip Solution:** Simplifies power design for mobile robots.

## 🔌 Wiring (I2C)

| IP5306 Pin | Raspberry Pi Pin | Description |
| :--- | :--- | :--- |
| **VCC** | 3.3V (Pin 1) | Logic Power |
| **GND** | GND (Pin 6) | Ground |
| **SDA** | GPIO 2 (Pin 3) | I2C Data |
| **SCL** | GPIO 3 (Pin 5) | I2C Clock |

*Note: Some variants of IP5306 do NOT support I2C. Ensure your chip is the I2C-enabled version (usually found on dedicated power modules).*

## 🚀 Usage

### 1. Launch the Node
```bash
ros2 run xpi_sensors ip5306_node
```

### 2. Verify Data
```bash
ros2 topic echo /ip5306_node/battery
```
*Output: `sensor_msgs/msg/BatteryState`*

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `i2c_bus` | int | `1` | I2C bus number. |
| `i2c_address` | int | `0x75` | I2C address (default 0x75). |
| `update_rate` | float | `1.0` | Polling rate in Hz. |

## 🧩 Topics Interface

### Publishers
*   `~/battery` (`sensor_msgs/msg/BatteryState`)
    *   `percentage`: Charge level (0.0 to 1.0).
    *   `power_supply_status`: Charging or Discharging.
    *   `voltage`: (Nominal 3.7V, precise reading not supported by this IC).
