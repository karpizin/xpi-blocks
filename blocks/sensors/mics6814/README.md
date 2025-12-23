# MICS-6814 Triple Gas Sensor (CO, NO2, NH3)

The **MICS-6814** is a robust MEMS sensor with three independent sensing elements in one package. It is specifically designed to detect **Carbon Monoxide (CO)**, **Nitrogen Dioxide (NO2)**, and **Ammonia (NH3)**.

**Note:** This is an analog sensor. Since the Raspberry Pi does not have analog inputs, you **must** use an ADC like the **ADS1115**.

## 🧠 Sensing Capabilities
1.  **RED Channel (Reducing gases):** Primarily measures **Carbon Monoxide (CO)**. Range: 1 – 1000ppm.
2.  **OX Channel (Oxidizing gases):** Primarily measures **Nitrogen Dioxide (NO2)**. Range: 0.05 – 10ppm.
3.  **NH3 Channel:** Measures **Ammonia (NH3)**. Range: 1 – 500ppm.

## 📦 Bill of Materials
*   Raspberry Pi
*   MICS-6814 Sensor Module (e.g., CJMCU-6814)
*   **ADS1115 I2C ADC**
*   Jumper Wires

## 🔌 Wiring
The MICS-6814 module typically has 3 analog outputs.

| MICS-6814 Pin | Function | Raspberry Pi / ADC | Note |
|---------------|----------|--------------------|-----------------------------------|
| **VCC**       | Power    | 5V (Pin 2)         | Heater needs 5V for stability.    |
| **GND**       | Ground   | GND (Pin 6)        | Common Ground                     |
| **CO / RED**  | Output 1 | **ADC A0**         | Analog voltage for CO             |
| **NO2 / OX**  | Output 2 | **ADC A1**         | Analog voltage for NO2            |
| **NH3**       | Output 3 | **ADC A2**         | Analog voltage for NH3            |

**Warning:** Ensure your ADC (ADS1115) is powered by the same voltage as your Pi's logic (3.3V) to protect the GPIOs, even if the sensor heater uses 5V.

## 🛠 Setup & Usage

This sensor is handled by the `analog_sensor_interpreter_node`.

1.  **Launch ADS1115 Node**:
    ```bash
    ros2 launch xpi_sensors ads1115.launch.py
    ```

2.  **Launch Interpreter** with MICS-6814 configuration for all 3 channels:
    ```bash
    ros2 run xpi_sensors analog_sensor_interpreter_node --ros-args -p sensor_configs_json:='[
      {"channel": 0, "type": "mics6814_co", "R0": 100000.0},
      {"channel": 1, "type": "mics6814_no2", "R0": 2200.0},
      {"channel": 2, "type": "mics6814_nh3", "R0": 150000.0}
    ]'
    ```

## 📡 Interface

### Publishers (via Interpreter)
*   `~/gas_co_ch0` (`std_msgs/Float32`): Carbon Monoxide in PPM.
*   `~/gas_no2_ch1` (`std_msgs/Float32`): Nitrogen Dioxide in PPM.
*   `~/gas_nh3_ch2` (`std_msgs/Float32`): Ammonia in PPM.

## 🔬 Calibration (R0)
The sensor measures the ratio `Rs/R0`. `R0` is the resistance of the sensor in **clean air**.
1.  Power the sensor and let the heater warm up for at least **30 minutes**.
2.  Place the sensor in a clean outdoor environment.
3.  Read the raw `Rs` (resistance) or calibrate the node to output a value.
4.  Update your `R0` parameters in the JSON configuration for each channel.

## ⚠️ Troubleshooting
*   **"High readings at startup"**: The MEMS heater needs time to reach operating temperature. Ignore the first 5-10 minutes of data.
*   **"Cross-sensitivity"**: Note that the NO2 sensor may react to Ozone (O3), and the CO sensor may react to Ethanol or Hydrogen. This is a characteristic of all metal-oxide sensors.
