# MAX17048: LiPo Battery Fuel Gauge

The **MAX17048** is a compact, low-power I2C fuel gauge for single-cell Li-ion/LiPo batteries. It provides much more accurate data than simple voltage monitoring.

**Key Features:**
*   **VCELL:** Battery voltage measurement.
*   **SOC (State of Charge):** Accurate percentage (0-100%).
*   **Crate:** Charge/Discharge rate.

## ⚡ Wiring (I2C)

| MAX17048 Pin | Raspberry Pi Pin | Description |
| :--- | :--- | :--- |
| **VCC** | 3.3V (Pin 1) | Power. |
| **GND** | GND (Pin 6) | Ground. |
| **SDA** | GPIO 2 (Pin 3) | I2C Data. |
| **SCL** | GPIO 3 (Pin 5) | I2C Clock. |

## 📦 Bill of Materials
*   1x MAX17048 Module (Adafruit or generic).
*   1x Single Cell LiPo Battery.

## 🚀 Usage

### 1. Launch the Node
```bash
ros2 launch xpi_sensors max17048.launch.py
```

### 2. Verify Data
```bash
ros2 topic echo /max17048/battery_state
```
*Output: `sensor_msgs/BatteryState` with `voltage` and `percentage` fields.*

## 🧩 Topics Interface

### Publishers
*   `~/battery_state` (`sensor_msgs/msg/BatteryState`)
    *   `voltage`: Current cell voltage (V).
    *   `percentage`: State of Charge (0.0 to 1.0).
    *   `power_supply_status`: Discharging/Charging (estimated).
