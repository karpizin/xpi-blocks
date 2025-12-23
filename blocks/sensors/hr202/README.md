# HR202 Humidity Sensor (Analog/Resistive)

The **HR202** is a low-cost resistive-type humidity sensor. Unlike digital sensors (like BME280), it acts as a variable resistor whose resistance decreases as humidity increases.

**Note:** Since the Raspberry Pi does not have built-in Analog-to-Digital Converters (ADC), this sensor **must** be used with an ADC like the **ADS1115**.

## 📦 Bill of Materials
*   Raspberry Pi
*   HR202 Sensor (Bare sensor or module)
*   **ADS1115 I2C ADC**
*   10kΩ Fixed Resistor (for voltage divider)
*   Jumper Wires

## 🔌 Wiring (Voltage Divider)
To measure resistance, we set up a voltage divider:

| Connection | Description |
|------------|-------------|
| **3.3V**   | Connect to one leg of HR202 |
| **ADS1115 A0** | Connect to the other leg of HR202 **AND** one leg of 10k resistor |
| **GND**    | Connect to the other leg of 10k resistor |

This creates a circuit: `3.3V -> HR202 -> ADC Input -> 10k Resistor -> GND`.

## 🛠 Setup & Usage

This sensor is handled by the `analog_sensor_interpreter_node`. 

1.  **Launch ADS1115 Node** to get raw voltages:
    ```bash
    ros2 launch xpi_sensors ads1115.launch.py
    ```

2.  **Launch Interpreter** with HR202 configuration:
    ```bash
    ros2 run xpi_sensors analog_sensor_interpreter_node --ros-args -p sensor_configs_json:='[{"channel": 0, "type": "hr202", "R_ref": 10000.0, "V_supply": 3.3}]'
    ```

## 📡 Interface

### Publishers (via Interpreter)
*   `~/humidity_ch0` (`sensor_msgs/RelativeHumidity`): Calculated relative humidity (0.0 to 1.0).

### Configuration Parameters (in JSON)
*   `channel`: ADC channel (0-3).
*   `type`: Must be `"hr202"`.
*   `R_ref`: Value of your fixed resistor in Ohms (default 10k).
*   `V_supply`: Supply voltage (usually 3.3).

## 🧠 Technical Note: AC vs DC
The HR202 is technically designed to be driven by an **AC signal** to prevent polarization and electrolysis of the sensor material, which can cause drift over time. 
*   This driver uses a simple **DC voltage divider** for simplicity, which is common in hobbyist projects. 
*   For high-precision or long-term industrial use, a dedicated AC-drive circuit (like using a 555 timer or a specific signal generator) is recommended.

## ⚠️ Troubleshooting
*   **Constant 0% or 100% Humidity:** 
    *   Check wiring. 
    *   Ensure the 10k resistor is correctly grounded.
*   **Erratic Readings:** 
    *   Resistive sensors are sensitive to noise. Keep wires short.
    *   Ensure a stable 3.3V supply.
