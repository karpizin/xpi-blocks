# MAX471: Voltage and Current Sensor

The **MAX471** module can measure both current (up to 3A) and voltage (up to 25V).
It is an analog sensor requiring an ADC (like ADS1115).

**Characteristics:**
*   **Current Output (AT):** 1V per Ampere (1V/A).
*   **Voltage Output (VT):** 1V per 5V Input (Ratio 1:5).

## ⚡ Wiring (via ADS1115)

| MAX471 Pin | Connection | Description |
| :--- | :--- | :--- |
| **RS+ / RS-** | Load Loop | Connect in series with load. |
| **GND** | GND | Common Ground. |
| **AT (Current)** | ADS1115 **A0** | Analog Current Output. |
| **VT (Voltage)** | ADS1115 **A1** | Analog Voltage Output. |

## 🚀 Usage

We use the generic `analog_sensor_interpreter_node` with a specific config.

### 1. Launch with Config
Create a launch file or use this JSON config structure:

```json
[
  {
    "channel": 0,
    "type": "max471",
    "sensitivity_mv_per_a": 1000, 
    "offset_voltage": 0.0
  },
  {
    "channel": 1,
    "type": "voltage_divider",
    "ratio": 5.0
  }
]
```

### 2. Verify Data
```bash
ros2 topic echo /analog/current_ch0
ros2 topic echo /analog/voltage_divider_ch1
```

## 🧩 Underlying Driver
This block uses:
*   `xpi_sensors/ads1115_node` (Hardware Driver)
*   `xpi_sensors/analog_sensor_interpreter_node` (Logic)
