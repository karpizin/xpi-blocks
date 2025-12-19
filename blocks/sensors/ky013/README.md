# KY-013: NTC Thermistor Temperature Sensor

The **KY-013** is a simple analog temperature sensor module containing an NTC thermistor and a 10k resistor in a voltage divider configuration.

## ⚡ Wiring (via ADS1115)

| KY-013 Pin | Connection | Description |
| :--- | :--- | :--- |
| **S (Signal)** | ADS1115 **A0** | Analog Output. |
| **Middle** | 3.3V / 5V | VCC (Reference Voltage). |
| **- (Minus)** | GND | Ground. |

> **Note:** Pinout varies by manufacturer. Check if "S" connects to the thermistor or the resistor. The `analog_sensor_interpreter` assumes a standard divider.

## 🚀 Usage

### 1. Config JSON
```json
[
  {
    "channel": 0,
    "type": "thermistor",
    "R_ref": 10000,
    "B_const": 3950,
    "V_supply": 3.3
  }
]
```

### 2. Verify Data
```bash
ros2 topic echo /analog/temperature_ch0
```
*Output: `sensor_msgs/Temperature` (Celsius).*

## 🧩 Underlying Driver
This block uses `xpi_sensors/analog_sensor_interpreter_node`.
