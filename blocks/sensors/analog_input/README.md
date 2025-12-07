# ADS1115 Analog-to-Digital Converter (ADC)

This block provides a ROS2 driver for the ADS1115, a 4-channel, 16-bit Analog-to-Digital Converter. It allows Raspberry Pi to read analog signals from various sensors and converts them into digital voltage readings.

## 📦 Bill of Materials
*   Raspberry Pi
*   ADS1115 module (I2C interface)
*   Jumper Wires
*   Analog sensors (e.g., potentiometers, LDRs, thermistors, current sensors, voltage dividers).

## 🔌 Wiring
Connect the ADS1115 module to the I2C pins on the Raspberry Pi. The address pins (ADDR) on the ADS1115 determine its I2C address.

| ADS1115 Pin | Raspberry Pi | Note |
|-------------|--------------|----------------------------------------------------------|
| VDD         | 3.3V         | Power for the module (Pin 1)                             |
| GND         | GND          | Common Ground (Pin 6)                                    |
| SDA         | GPIO 2 (SDA) | Data line (Pin 3)                                        |
| SCL         | GPIO 3 (SCL) | Clock line (Pin 5)                                       |
| ADDR        | GND/VDD/SDA/SCL | Sets I2C address: 0x48 (GND), 0x49 (VDD), 0x4A (SDA), 0x4B (SCL) |
| A0, A1, A2, A3 | Analog Sensor Out | Connect analog sensor outputs here                       |

**Important I2C Configuration on Raspberry Pi:**
You must enable the I2C interface on your Raspberry Pi.

1.  **Enable I2C:**
    ```bash
    sudo raspi-config
    # Interface Options -> P3 I2C -> Yes
    ```
2.  **Verify I2C Address:** Check the ADS1115's I2C address (e.g., 0x48 if ADDR pin connected to GND)
    ```bash
    sudo apt install i2c-tools
    i2cdetect -y 1 # Or 0 if using an older Pi
    # Look for '48' (or your configured address) in the output.
    ```

## 🚀 Quick Start
1.  **Perform I2C Configuration** as described above.
2.  **Launch the ADS1115 driver**:
    ```bash
    # Example: Read from channel A0 with default PGA (+/-6.144V)
    ros2 launch xpi_sensors ads1115.launch.py channels:="[0]" i2c_address:=0x48

    # Example: Read from channels A0 and A1 with a higher PGA (+/-2.048V) for smaller voltages
    ros2 launch xpi_sensors ads1115.launch.py channels:="[0, 1]" pga:="2" i2c_address:=0x48
    ```
    *Note: The `channels` argument must be passed as a JSON-formatted string representing a list.*

## 📡 Interface
### Publishers
*   `~/voltage_ch<CHANNEL_NUMBER>` (`std_msgs/Float32`): Publishes the measured voltage for each monitored channel.
    *   Example: `/ads1115_adc/voltage_ch0`
*   `~/all_voltages` (`std_msgs/Float32MultiArray`): Publishes an array of voltages for all monitored channels, in the order specified in the `channels` parameter. Only published if multiple channels are configured.

### Parameters
*   `i2c_bus` (int, default: `1`): I2C bus number.
*   `i2c_address` (int, default: `0x48`): I2C address of the ADS1115.
*   `publish_rate` (float, default: `10.0`): Frequency to publish data in Hz.
*   `channels` (list of int, default: `[0]`): List of single-ended channels to read (0-3).
*   `pga` (string, default: `"2/3"`): Programmable Gain Amplifier setting.
    *   `"2/3"`: +/-6.144V (good for up to 5V inputs)
    *   `"1"`: +/-4.096V
    *   `"2"`: +/-2.048V
    *   `"4"`: +/-1.024V
    *   `"8"`: +/-0.512V
    *   `"16"`: +/-0.256V
*   `data_rate` (int, default: `128`): Samples per second (8, 16, 32, 64, 128, 250, 475, 860).
*   `frame_id` (string, default: `ads1115_link`): Frame ID for messages.
*   `mock_hardware` (bool, default: `false`): Run in mock mode without real ADS1115 hardware.

## ✅ Verification
1.  Launch the driver with your ADS1115 connected and I2C enabled.
2.  In new terminals, monitor the published topics:
    ```bash
    ros2 topic echo /ads1115_adc/voltage_ch0
    ros2 topic echo /ads1115_adc/all_voltages
    ```
    You should see streams of voltage readings. Vary the analog input to see changes.

## 💡 Examples of Analog Sensors
The ADS1115 acts as a bridge for many types of analog sensors:

### 1. Potentiometer (Variable Resistor)
*   **Wiring:** Connect one end to 3.3V, other end to GND. Wiper to one of ADS1115 analog input (A0-A3).
*   **Interpretation:** Voltage output varies directly with knob position.

### 2. Photoresistor (LDR - Light Dependent Resistor)
*   **Wiring:** Create a voltage divider: 3.3V -> LDR -> ADS1115 Analog Input -> Fixed Resistor (e.g., 10kΩ) -> GND.
*   **Interpretation:** Voltage changes with light intensity.

### 3. Thermistor (Temperature Dependent Resistor)
*   **Wiring:** Similar to LDR, create a voltage divider with a fixed resistor.
*   **Interpretation:** Voltage changes with temperature. Requires a lookup table or a formula (Steinhart-Hart) to convert voltage to temperature.

### 4. ACS712 Current Sensor
*   **Wiring:** ACS712 VCC to 5V, GND to GND. Output pin to ADS1115 Analog Input.
*   **Interpretation:** The sensor outputs a voltage proportional to the current flowing through it. Requires calibration to convert voltage to Amperes. Choose PGA to match the sensor's output range (e.g., +/-5A version outputs +/-1.5V, so PGA "2" or "1" might be suitable).

### 5. Voltage Divider
*   **Wiring:** To measure voltages higher than 3.3V (but up to the Pi's absolute max, typically 5V), create a voltage divider. For example, to measure a 5V source, use two equal resistors (e.g., 10kΩ and 10kΩ) to divide the voltage by 2. Connect the divided voltage to an ADS1115 Analog Input.
*   **Interpretation:** Read the divided voltage, then multiply by the division ratio to get the actual source voltage.

## ⚠️ Troubleshooting
*   **"Failed to initialize ADS1115" / "I/O Error"**:
    *   Double-check I2C wiring (SDA/SCL, VDD/GND).
    *   Verify I2C is enabled (`sudo raspi-config`).
    *   Ensure the correct `i2c_address` is used (check ADDR pin connection).
    *   Check for I2C permissions (`sudo usermod -a -G i2c $USER`).
*   **No data / erratic data**:
    *   Check physical connections.
    *   Verify module is powered.
    *   Ensure proper grounding.
    *   Adjust `pga` if input voltage is outside the selected range (leads to saturation).

---

# Analog Sensor Interpreter

This block provides a ROS2 node that subscribes to raw voltage outputs from an ADS1115 (or similar ADC) and interprets these voltages into meaningful physical units (e.g., temperature, illuminance, current, soil moisture, etc.) based on user-defined sensor configurations.

## 📦 Bill of Materials
*   Raspberry Pi
*   Running `xpi_sensors/ads1115_node`
*   Various analog sensors connected to the ADS1115 (thermistors, LDRs, current sensors, soil moisture, etc.)

## 🚀 Quick Start
1.  **Launch the ADS1115 driver:**
    ```bash
    # Example: Reading voltages from ADS1115 channels 0 and 1
    ros2 launch xpi_sensors ads1115.launch.py channels:="[0, 1]"
    ```
2.  **Prepare Sensor Configurations:**
    *   Create a JSON string that describes each analog sensor connected to the ADS1115.
    *   This JSON string will be passed as the `sensor_configs_json` parameter.

    **Example `sensor_configs_json` for a Thermistor on Ch 0 and LDR on Ch 1:**
    ```json
    [
        {
            "channel": 0,
            "type": "thermistor",
            "R_ref": 10000.0,
            "B_const": 3950.0,
            "V_supply": 3.3
        },
        {
            "channel": 1,
            "type": "ldr",
            "R_ref": 10000.0,
            "V_supply": 3.3,
            "lux_factor": 500000.0
        }
    ]
    ```
    *   **Common `config` fields for sensors:**
        *   `channel` (int, required): ADS1115 channel (0-3).
        *   `type` (string, required): Sensor type (`thermistor`, `ldr`, `acs712`, `voltage_divider`, `soil_moisture`, etc.).
        *   `V_supply` (float, optional, default: 3.3): Supply voltage for voltage dividers.
    *   **`thermistor` specific fields:**
        *   `R_ref` (float, default: 10000.0): Reference resistor in series with thermistor.
        *   `B_const` (float, default: 3950.0): Thermistor Beta constant.
        *   `T_nominal` (float, default: 298.15): Nominal temperature in Kelvin (25°C).
    *   **`ldr` specific fields:**
        *   `R_ref` (float, default: 10000.0): Reference resistor in series with LDR.
        *   `lux_factor` (float, default: 500000.0): Calibration factor for lux conversion (highly sensor-dependent).
    *   **`acs712` specific fields:**
        *   `sensitivity_mv_per_a` (float, default: 185.0): Sensitivity in mV/A.
        *   `offset_voltage` (float, default: 2.5): Output voltage at 0A.
    *   **`voltage_divider` specific fields:**
        *   `ratio` (float, default: 2.0): Total voltage / measured voltage.
    *   **`soil_moisture` specific fields:**
        *   `min_voltage` (float): Voltage reading when sensor is driest.
        *   `max_voltage` (float): Voltage reading when sensor is wettest.
    *   **`temt6000`, `guva_s12`, `ml8511` specific fields:**
        *   `offset_voltage` (float, default: 0.0)
        *   `scale_factor` (float, default: 1.0)

3.  **Launch the Analog Sensor Interpreter Node:**
    ```bash
    # Example using the JSON above via an environment variable
    export XPI_SENSOR_CONFIGS='[{"channel": 0, "type": "thermistor", "R_ref": 10000.0, "B_const": 3950.0, "V_supply": 3.3}, {"channel": 1, "type": "ldr", "R_ref": 10000.0, "V_supply": 3.3, "lux_factor": 500000.0}]'
    ros2 launch xpi_sensors analog_sensor_interpreter.launch.py \
        sensor_configs_json:="${XPI_SENSOR_CONFIGS}" adc_node_name:=ads1115_adc
    ```

## 📡 Interface
### Subscribers
*   `/<adc_node_name>/voltage_ch<CHANNEL_NUMBER>` (`std_msgs/Float32`): Voltage readings from the ADC for each configured channel.

### Publishers (dynamic based on `sensor_configs_json`)
*   `~/temperature_ch<CHANNEL_NUMBER>` (`sensor_msgs/Temperature`): For `thermistor` type.
*   `~/illuminance_ch<CHANNEL_NUMBER>` (`sensor_msgs/Illuminance`): For `ldr`, `temt6000`, `guva_s12`, `ml8511` types.
*   `~/current_ch<CHANNEL_NUMBER>` (`std_msgs/Float32`): For `acs712`, `max471` types (Amperes).
*   `~/voltage_divider_ch<CHANNEL_NUMBER>` (`std_msgs/Float32`): For `voltage_divider` type (Volts).
*   `~/soil_moisture_ch<CHANNEL_NUMBER>` (`std_msgs/Float32`): For `soil_moisture` type (%).
*   `~/analog_range_ch<CHANNEL_NUMBER>` (`sensor_msgs/Range`): For `vibration`, `noise_level` types (raw voltage).

### Parameters
*   `adc_node_name` (string, default: `ads1115_adc`): ROS2 name of the ADS1115 node publishing voltages.
*   `publish_rate` (float, default: `10.0`): Frequency to publish interpreted sensor readings in Hz.
*   `frame_id` (string, default: `analog_sensors_link`): Frame ID for messages.
*   `sensor_configs_json` (string, default: `'[]'`): JSON string defining sensor types and calibration parameters.

## ✅ Verification
1.  Launch `ads1115.launch.py` and `analog_sensor_interpreter.launch.py` with your configurations.
2.  Monitor the interpreted topics (e.g., `/analog_sensor_interpreter/temperature_ch0`).
    ```bash
    ros2 topic echo /analog_sensor_interpreter/temperature_ch0
    ```
3.  Vary the physical input (e.g., temperature for a thermistor, light for LDR) and observe the interpreted values.

## ⚠️ Troubleshooting
*   **No interpreted data?**
    *   Verify `ads1115_node` is running and publishing on the correct topics.
    *   Check `analog_sensor_interpreter_node` logs for JSON parsing errors or "Unsupported sensor type" warnings.
    *   Ensure `adc_node_name` parameter in the interpreter matches the name of your ADS1115 node.
    *   Review your `sensor_configs_json` for syntax errors or incorrect `channel` numbers.
*   **Incorrect readings?**
    *   Review calibration parameters (`R_ref`, `B_const`, `sensitivity_mv_per_a`, etc.) for your specific sensor.
    *   Ensure `V_supply` matches the actual voltage used in your voltage divider circuits.
    *   Adjust ADS1115 `pga` settings for optimal voltage range.

