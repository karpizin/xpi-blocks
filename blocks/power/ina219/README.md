# INA219: High Side DC Current and Voltage Sensor

The **INA219** is an I2C zerø-drift, bidirectional current/power monitor. It can measure:
1.  **Bus Voltage:** Voltage of your battery/supply (up to 26V).
2.  **Shunt Voltage:** Voltage drop across the shunt resistor.
3.  **Current:** Amps flowing through the load.
4.  **Power:** Power consumption in mW.

**Robotics Use Case:**
*   Monitor battery level (V).
*   Detect stalled motors (High Current).
*   Calculate total energy consumption.

## ⚡ Wiring (I2C)

| INA219 Pin | Raspberry Pi Pin | Description          |
| :---       | :---             | :---                 |
| **VCC**    | 3.3V (Pin 1)     | Power (3.3V)         |
| **GND**    | GND (Pin 6)      | Ground               |
| **SDA**    | GPIO 2 (Pin 3)   | I2C Data             |
| **SCL**    | GPIO 3 (Pin 5)   | I2C Clock            |
| **Vin+**   | Battery (+)      | Input voltage        |
| **Vin-**   | Load (+)         | To Motor/Robot       |

## 📦 Bill of Materials
*   1x INA219 Module (Usually has a 0.1 Ohm shunt)
*   4x Jumper Wires

## 🚀 Usage

### 1. Launch the Node
```bash
ros2 launch xpi_sensors ina219.launch.py
```

### 2. Verify Data
```bash
ros2 topic echo /ina219/battery_state
```
*Output: `sensor_msgs/BatteryState` containing voltage, current, etc.*

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `i2c_bus` | int | `1` | I2C bus number. |
| `i2c_address` | int | `0x40` | Default address (0x40 - 0x4F configurable). |
| `polling_rate` | float | `10.0` | Hz. |
| `shunt_ohms` | float | `0.1` | Value of shunt resistor (default modules use 0.1). |
| `max_amps` | float | `3.2` | Expected max current for calibration. |

## 🧩 Topics Interface

### Publishers
*   `~/battery_state` (`sensor_msgs/msg/BatteryState`)
    *   `voltage`: Bus Voltage (V).
    *   `current`: Current (A). Negative value means discharging.
    *   `charge`: Power (W) (repurposed field if capacity is unknown).
    *   `location`: "ina219".
