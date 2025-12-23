# DHT11 / DHT22 (AM2302) Temperature and Humidity Sensor

These are very popular, low-cost digital temperature and humidity sensors. They use a proprietary single-wire protocol that requires precise timing.

**Note on Stability:** Due to the non-real-time nature of Linux (Raspberry Pi OS), reading these sensors can result in frequent checksum (CRC) errors. This driver handles them by retrying/skipping frames, but don't expect 100% success rate on every poll.

## 📦 Bill of Materials
*   Raspberry Pi
*   DHT11 or DHT22 (AM2302) Module
*   4.7kΩ - 10kΩ Resistor (Pull-up) - *Required if not built into your module*.
*   Jumper Wires

## 🔌 Wiring
Connect the sensor to a GPIO pin (default GPIO 4).

| DHT Pin | Raspberry Pi | Note |
|---------|--------------|-----------------------------------|
| VCC     | 3.3V or 5V   | Power (Pin 1). 3.3V recommended for RPi logic safety. |
| DATA    | GPIO 4       | Data Signal (Pin 7). Needs Pull-up resistor to VCC! |
| NC      | -            | Not Connected |
| GND     | GND          | Ground (Pin 6) |

**Pull-up Resistor:** Connect a resistor (4.7k - 10k Ohm) between **VCC** and **DATA**. Many "PCB Modules" (3 pins) already have this soldered. If you have the bare 4-pin sensor, you must add it.

## 🛠 Software Setup

1.  **Install System Dependencies:**
    Required for `libgpiod` access.
    ```bash
    sudo apt-get install python3-libgpiod
    ```

2.  **Install Python Library:**
    ```bash
    pip3 install adafruit-circuitpython-dht
    ```

## 🚀 Usage

**Launch for DHT22 (Default):**
```bash
ros2 launch xpi_sensors dht.launch.py
```

**Launch for DHT11 on GPIO 17:**
```bash
ros2 launch xpi_sensors dht.launch.py sensor_type:=DHT11 gpio_pin:=17
```

## 📡 Interface

### Publishers
*   `~/temperature` (`sensor_msgs/Temperature`): Ambient temperature in Celsius.
*   `~/humidity` (`sensor_msgs/RelativeHumidity`): Relative humidity (0.0 to 1.0).

### Parameters
*   `sensor_type` (string, default: `DHT22`): `DHT11` or `DHT22`.
*   `gpio_pin` (int, default: `4`): BCM GPIO pin number.
*   `publish_rate` (float, default: `0.5`): **Important!** 
    *   DHT22 requires at least 2 seconds between reads (Max 0.5Hz).
    *   DHT11 requires at least 1 second (Max 1.0Hz).
    *   Setting this higher will cause read failures.
*   `frame_id` (string, default: `dht_link`): Frame ID for TF.

## ⚠️ Troubleshooting
*   **"DHT Read Error (Normal): Checksum did not match"**: 
    *   This is very common on Linux. The driver logs this as debug/info and waits for the next cycle. It's not a critical failure unless it happens 100% of the time.
*   **"Unable to set line offset"**: 
    *   The GPIO pin might be in use by another process or claimed by another driver.
*   **"Pulse not detected"**:
    *   Check wiring.
    *   Ensure Pull-up resistor is present.
    *   Check power (3.3V vs 5V). Some DHT11s need 5V, but be careful with RPi GPIO (it's 3.3V logic). Use a level shifter or 3.3V if possible.
