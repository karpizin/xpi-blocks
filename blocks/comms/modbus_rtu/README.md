# Modbus RTU (RS-485) Generic Driver

This block implements a configurable **Modbus RTU Master**.
It allows you to read registers from industrial sensors, PLCs, solar controllers, and energy meters without writing Python code.

You define a "Register Map" in the launch configuration, and this node publishes the data to ROS2.

## ⚡ Wiring (RS-485)

Raspberry Pi typically requires a **USB-to-RS485** adapter or an RS485 HAT (UART).

| RS-485 Adapter | Device (Slave) |
| :--- | :--- |
| **A+** | A+ |
| **B-** | B- |
| **GND** | GND (Recommended) |

## 📦 Bill of Materials
*   1x USB-RS485 Adapter (~$2)
*   Modbus Device (e.g., Soil Moisture, Relay, Energy Meter)

## 🚀 Usage

### 1. Configure Registers
Edit the launch file or parameters. Example config for a Soil Moisture Sensor:
```json
[
  {
    "name": "moisture",
    "register": 0,
    "count": 1,
    "scale": 0.1,
    "function_code": 3
  },
  {
    "name": "temperature",
    "register": 1,
    "count": 1,
    "scale": 0.1,
    "function_code": 3
  }
]
```

### 2. Launch
```bash
ros2 launch xpi_comms modbus_rtu.launch.py
```

### 3. Verify Data
```bash
ros2 topic echo /modbus_rtu/data
```
*Output: `{"moisture": 25.4, "temperature": 22.1}`*

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `port` | string | `/dev/ttyUSB0` | Serial port. |
| `baudrate` | int | `9600` | Communication speed. |
| `slave_address` | int | `1` | Modbus Slave ID. |
| `register_map` | string | `[]` | JSON array of registers to poll. |
| `polling_rate` | float | `1.0` | Hz. |

## 🧩 Topics Interface

### Publishers
*   `~/data` (`std_msgs/msg/String`) - JSON string containing all polled values.
