# GPS/GNSS: NMEA Serial Driver

This block enables GPS/GLONASS/Galileo tracking using any **NMEA-compatible** serial module (e.g., u-blox NEO-M8N, Beitian BN-220).

## ⚙️ System Setup (Mandatory)

By default, Raspberry Pi uses the primary UART for the serial console. You must disable it:

1.  Run `sudo raspi-config`.
2.  Go to **Interface Options** -> **Serial Port**.
3.  **Would you like a login shell to be accessible over serial?** -> **No**.
4.  **Would you like the serial port hardware to be enabled?** -> **Yes**.
5.  Reboot the Pi.

The GPS will usually be available at `/dev/ttyS0` (Pi 3/4/Zero W) or `/dev/ttyAMA0`.

## ⚡ Wiring (UART)

| GPS Module Pin | Raspberry Pi Pin | Description |
| :---           | :---             | :---                 |
| **VCC**        | 3.3V or 5V       | Check your module!   |
| **GND**        | GND (Pin 6)      | Ground               |
| **TX**         | GPIO 15 (Pin 10) | UART RX (RPi RX)     |
| **RX**         | GPIO 14 (Pin 8)  | UART TX (Optional)   |

## 🚀 Usage

### 1. Launch the Node
```bash
ros2 launch xpi_sensors gps.launch.py
```

### 2. Verify Data
```bash
ros2 topic echo /gps/fix
```
*Output: Latitude, Longitude, Altitude, and Status.*

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `port` | string | `/dev/ttyS0` | Serial port path. |
| `baudrate` | int | `9600` | Communication speed (default for most GPS). |
| `frame_id` | string | `gps_link` | TF frame for the sensor. |

## 🧩 Topics Interface

### Publishers
*   `~/fix` (`sensor_msgs/msg/NavSatFix`) - Global coordinates and status.
*   `~/time_reference` (`sensor_msgs/msg/TimeReference`) - Precise GPS time.
