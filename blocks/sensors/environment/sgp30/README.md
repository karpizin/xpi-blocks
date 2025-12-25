# SGP30 Gas Sensor (TVOC / eCO2)

The SGP30 is a high-performance MOx (Metal-Oxide) gas sensor with multiple sensing elements on a single chip. It provides total volatile organic compounds (TVOC) and equivalent CO2 (eCO2) readings.

## 🧠 Why SGP30?
Unlike cheaper MOx sensors, the SGP30 features **dynamic baseline compensation** and **humidity compensation**, making it one of the most reliable choices for indoor air quality monitoring in robotics.

## 📦 Bill of Materials
*   Raspberry Pi (4, 5, or Zero)
*   SGP30 Sensor Module (I2C)
*   Jumper Wires

## 🔌 Wiring

| SGP30 Pin | Raspberry Pi | Note |
|-----------|--------------|---------------------------|
| VCC       | 3.3V (Pin 1) | |
| GND       | GND (Pin 6)  | |
| SDA       | SDA (Pin 3)  | I2C Data |
| SCL       | SCL (Pin 5)  | I2C Clock |

## 🛠 Software Setup

1.  **Enable I2C:** `sudo raspi-config` -> Interface Options -> I2C.
2.  **Dependencies:**
    ```bash
    pip install adafruit-circuitpython-sgp30
    ```

## 🚀 Usage

**Launch the node:**
```bash
ros2 run xpi_sensors sgp30_node
```

## 📡 Interface

### Publishers
*   `~/eco2` (`std_msgs/Int32`): Equivalent CO2 in **ppm** (parts per million).
*   `~/tvoc` (`std_msgs/Int32`): Total Volatile Organic Compounds in **ppb** (parts per billion).
*   `~/raw_signals` (`std_msgs/Float32MultiArray`): Raw Hydrogen and Ethanol signals.

## ⚠️ Warm-up and Calibration
*   **Warm-up:** The sensor requires approximately 15 seconds after power-up before it returns valid IAQ (Indoor Air Quality) data.
*   **Baseline:** The SGP30 maintains an internal baseline. For best results, the sensor should be run continuously.
