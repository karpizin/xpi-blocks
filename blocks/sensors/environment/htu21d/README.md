# HTU21D / SHT20 Temperature & Humidity Sensors

The HTU21D and SHT20 are widely used I2C sensors providing high-accuracy temperature and relative humidity measurements. They are interchangeable from a software perspective.

## 🧠 Why HTU21D/SHT20?
*   **Small Form Factor:** Usually comes in very compact breakout boards.
*   **Reliable:** Great alternative to DHT sensors for I2C-based systems.
*   **Standard Interface:** Uses standard ROS2 messages for environment monitoring.

## 📦 Bill of Materials
*   Raspberry Pi (4, 5, or Zero)
*   HTU21D or SHT20 Sensor Module (I2C)
*   Jumper Wires

## 🔌 Wiring

| Sensor Pin | Raspberry Pi | Note |
|------------|--------------|---------------------------|
| VIN        | 3.3V (Pin 1) | |
| GND        | GND (Pin 6)  | |
| SDA        | SDA (Pin 3)  | I2C Data |
| SCL        | SCL (Pin 5)  | I2C Clock |

## 🛠 Software Setup

1.  **Enable I2C:** `sudo raspi-config` -> Interface Options -> I2C.
2.  **Dependencies:**
    ```bash
    pip install adafruit-circuitpython-htu21d
    ```

## 🚀 Usage

**Launch the node:**
```bash
ros2 run xpi_sensors htu21d_node
```

## 📡 Interface

### Publishers
*   `~/temperature` (`sensor_msgs/Temperature`): Ambient temperature in Celsius.
*   `~/humidity` (`sensor_msgs/RelativeHumidity`): Relative humidity (0.0 to 1.0).

### Parameters
*   `polling_rate` (float, default: `1.0`): Frequency of measurements in Hz.
*   `i2c_bus` (int, default: `1`): I2C bus index.
