# LTE / 4G Modem Management (SIM7600)

This block provides ROS2 integration for LTE modems, specifically the **SIM7600G-H** series often used as Raspberry Pi HATs. It allows your robot to communicate over cellular networks and provides management tools for SMS and GPS.

## 🛰 Features
1.  **Internet Connectivity:** Configuration guide for `wwan0` (LTE data).
2.  **Signal Monitoring:** Publishes RSSI and network type (4G/3G/2G).
3.  **SMS Bridge:** Send and receive SMS messages via ROS2 topics.
4.  **Integrated GNSS:** Publishes GPS coordinates from the modem's built-in receiver.

## 📦 Bill of Materials
*   Raspberry Pi
*   SIM7600G-H 4G HAT (e.g., Waveshare)
*   4G Antenna + GPS Antenna
*   SIM Card (Active data plan)
*   **External 5V 3A Power Supply** (Modems consume high current during transmission).

## 🔌 Hardware Setup
1.  Connect the HAT to the Pi's 40-pin header.
2.  Connect the **microUSB** port of the HAT to one of the Pi's USB ports (This is how the AT/Data ports are exposed).
3.  Insert SIM card.
4.  Toggle the power switch on the HAT.

## 🛠 Software Setup (Linux Networking)

Before running the ROS2 node, you must configure the modem as a network interface.

### Option 1: Using NetworkManager (Easiest)
```bash
sudo nmcli connection add type gsm ifname cdc-wdm0 con-name lte-data apn YOUR_APN
sudo nmcli connection up lte-data
```
*(Replace `YOUR_APN` with your provider's APN, e.g., 'internet').*

### Option 2: RAW USB Ports
The modem exposes multiple ports via USB:
*   `/dev/ttyUSB1`: Diagnostic port.
*   `/dev/ttyUSB2`: **AT Command port** (Used by the ROS2 node).
*   `/dev/ttyUSB3`: NMEA/Data port.

## 🚀 ROS2 Usage

**Launch the management node:**
```bash
ros2 launch xpi_comms lte_modem.launch.py serial_port:=/dev/ttyUSB2
```

## 📡 Interface

### Publishers
*   `~/rssi` (`std_msgs/Int32`): Signal strength (0-31). 15+ is stable, 25+ is excellent.
*   `~/network_type` (`std_msgs/String`): "4G/LTE", "3G", etc.
*   `~/gps` (`sensor_msgs/NavSatFix`): GNSS coordinates.
*   `~/sms_received` (`std_msgs/String`): Content of incoming SMS.

### Subscribers
*   `~/send_sms` (`std_msgs/String`): Format: `"PHONE_NUMBER:MESSAGE"`.
    *   Example: `ros2 topic pub /lte_management/send_sms std_msgs/String "data: '+123456789:Robot Online!'"`

## ⚠️ Troubleshooting
*   **"SerialException: device not found"**: Ensure the microUSB cable is connected between the HAT and the Pi.
*   **Modem restarts or Pi hangs**: Modems draw high current peaks. Ensure you are using a high-quality 5V/3A+ power supply.
*   **No GPS Fix**: Ensure the GPS antenna is connected and has a clear view of the sky.
