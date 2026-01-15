# ACS712: Analog Current Sensor (Hall Effect)

The **ACS712** measures AC or DC current using the Hall effect. It provides an analog voltage output proportional to the current.
Since Raspberry Pi has no analog inputs, this sensor requires an **ADC (like ADS1115)**.

**Models & Sensitivity:**
*   **5A Model:** 185 mV/A
*   **20A Model:** 100 mV/A
*   **30A Model:** 66 mV/A

**Zero Point:** Output is **VCC/2** (approx 2.5V) when current is 0A.

## ⚡ Wiring (via ADS1115)

| ACS712 Pin | Connection | Description |
| :--- | :--- | :--- |
| **VCC** | 5V | Needs 5V logic usually. |
| **GND** | GND | Common Ground. |
| **OUT** | ADS1115 **A0** | Analog Output (0-5V). |

> **⚠️ Voltage Warning:** The output can go up to 5V. The ADS1115 handles this fine (if powered by 5V or using input dividers), but do NOT connect directly to a Pi GPIO. Using ADS1115 is safe.

## 📦 Bill of Materials
*   1x ACS712 Module (5A/20A/30A)
*   1x ADS1115 ADC Module
*   Jumper Wires

## 🚀 Usage

We use the generic `analog_sensor_interpreter_node` to drive this.

### 1. Launch with Pre-config
```bash
ros2 launch xpi_sensors acs712.launch.py
```

### 2. Verify Data
```bash
ros2 topic echo /acs712/current
```
*Output: `sensor_msgs/BatteryState` (current field).*

## ⚙️ Logic (How it works)
The interpreter node uses a linear mapping formula:
`Current (A) = (Voltage - Zero_Offset) / Sensitivity`

*   **Zero_Offset:** ~2.5V (depends on supply).
*   **Sensitivity:** 0.185, 0.100, or 0.066 (V/A).

## 🧩 Topics Interface

### Publishers
*   `~/current` (`sensor_msgs/msg/BatteryState`) - Reports Current in Amps.
