# SW6106: Fast Charge Power Bank IC (I2C)

The **SW6106** is a highly integrated power management IC for fast-charge power banks. It is commonly found in the **Waveshare Li-ion Battery HAT** for Raspberry Pi. It provides I2C telemetry for monitoring battery health and charging status.

## ⚡ Features
*   **Battery Level:** Real-time percentage (0-100%).
*   **Voltage:** Precise battery voltage monitoring (mV).
*   **Current:** Charge/Discharge current monitoring (mA).
*   **Protection:** Over-charge, over-discharge, and short-circuit protection (hardware-level).

## 🔌 Wiring (I2C)

| SW6106 Pin | Raspberry Pi Pin | Description |
| :--- | :--- | :--- |
| **VCC** | 3.3V (Pin 1) | Logic Power |
| **GND** | GND (Pin 6) | Ground |
| **SDA** | GPIO 2 (Pin 3) | I2C Data |
| **SCL** | GPIO 3 (Pin 5) | I2C Clock |

*Note: On most HATs, the I2C pins are already connected through the 40-pin header.*

## 🚀 Usage

### 1. Launch the Node
```bash
ros2 run xpi_sensors sw6106_node
```

### 2. Verify Data
```bash
ros2 topic echo /sw6106_node/battery
```
*Output: `sensor_msgs/msg/BatteryState`*

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `i2c_bus` | int | `1` | I2C bus number. |
| `i2c_address` | int | `0x3C` | I2C address (default 0x3C). |
| `update_rate` | float | `1.0` | Polling rate in Hz. |

## 🧩 Topics Interface

### Publishers
*   `~/battery` (`sensor_msgs/msg/BatteryState`)
    *   `voltage`: Battery voltage in Volts.
    *   `current`: Current in Amperes.
    *   `percentage`: Charge level (0.0 to 1.0).
    *   `power_supply_status`: Charging or Discharging.
