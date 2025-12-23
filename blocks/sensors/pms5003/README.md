# PMS5003 / PMS7003 Particulate Matter Sensor

The **Plantower PMS5003** (and its siblings PMS7003/PMS1003) is a digital universal particle concentration sensor. It uses **laser scattering** to detect suspended particles in the air, specifically **PM1.0, PM2.5, and PM10**.

## 🧠 What do these numbers mean?
Particulate Matter (PM) is measured in micrograms per cubic meter (µg/m³).
*   **PM1.0:** Ultra-fine particles (<1.0µm). Viruses, smoke, combustion particles. Can enter bloodstream.
*   **PM2.5:** Fine particles (<2.5µm). Combustion, organic compounds, metals. Can penetrate deep into lungs. **The main Air Quality Index (AQI) metric.**
*   **PM10:** Coarse particles (<10µm). Dust, pollen, mold. Irritates eyes/nose.

## 📦 Bill of Materials
*   Raspberry Pi
*   PMS5003 or PMS7003 Sensor
*   JST-GH 1.25mm 8-pin cable breakout (often comes with the sensor)
*   Jumper Wires

## 🔌 Wiring (UART)
The sensor communicates via Serial (UART) at **9600 baud**.

| PMS5003 Pin | Function | Raspberry Pi | Note |
|-------------|----------|--------------|-----------------------------------|
| 1 (VCC)     | 5V       | 5V (Pin 2/4) | **Sensor needs 5V power!** |
| 2 (GND)     | GND      | GND (Pin 6)  | Common Ground |
| 3 (SET)     | Control  | 3.3V         | Pull High for Active Mode |
| 4 (RX)      | RXD      | -            | Not strictly needed for reading |
| 5 (TX)      | TXD      | GPIO 15 (RX) | **Connect to Pi's RX pin (Pin 10)** |
| 6 (RESET)   | Reset    | 3.3V         | Pull High to run |

**Important:**
1.  **Logic Levels:** The PMS5003 TX line is 3.3V logic compatible, so it is safe to connect directly to the Raspberry Pi's RX pin.
2.  **Serial Port:** You must enable the UART serial port on the Pi.
    ```bash
    sudo raspi-config
    # Interface Options -> Serial Port
    # Login shell: NO
    # Serial hardware: YES
    ```
    Reboot after changing this. This usually enables `/dev/ttyS0` or `/dev/ttyAMA0`.

## 🚀 Usage

**Launch with default settings (/dev/ttyAMA0):**
```bash
ros2 launch xpi_sensors pms5003.launch.py
```

**Launch with USB Serial Adapter (/dev/ttyUSB0):**
```bash
ros2 launch xpi_sensors pms5003.launch.py serial_port:=/dev/ttyUSB0
```

## 📡 Interface

### Publishers
*   `~/pm1_0` (`std_msgs/Float32`): PM1.0 concentration (µg/m³) [Atmospheric Env].
*   `~/pm2_5` (`std_msgs/Float32`): PM2.5 concentration (µg/m³) [Atmospheric Env].
*   `~/pm10` (`std_msgs/Float32`): PM10 concentration (µg/m³) [Atmospheric Env].
*   `~/data` (`std_msgs/Float32MultiArray`): `[PM1.0, PM2.5, PM10]` for easy plotting.

### Parameters
*   `serial_port` (string, default: `/dev/ttyAMA0`): The device path.
*   `baudrate` (int, default: `9600`): Fixed for PMS5003.
*   `polling_rate` (float, default: `1.0`): Hz.
*   `frame_id` (string, default: `pms5003_link`): Frame ID for TF.
*   `mock_hardware` (bool, default: `false`): Simulate data if no sensor attached.

## ⚠️ Troubleshooting
*   **"Permission denied: /dev/ttyAMA0"**:
    *   Add your user to the `dialout` group: `sudo usermod -a -G dialout $USER`.
    *   Log out and log back in.
*   **"No data"**:
    *   Check wiring TX (Sensor) -> RX (Pi).
    *   Ensure SET and RESET pins are pulled high (connected to 3.3V).
