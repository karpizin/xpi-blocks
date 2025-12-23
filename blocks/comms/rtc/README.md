# Real-Time Clock (RTC) Modules: DS3231 / DS1307

An **RTC** module allows the Raspberry Pi to maintain accurate time even when powered off and without an internet connection. This is essential for logging, scheduling, and autonomous navigation.

## 🧠 Supported Modules
*   **DS3231 (Recommended):** High-precision temperature-compensated RTC with an accuracy of ±2ppm. It also includes an internal temperature sensor.
*   **DS1307:** Lower cost, less accurate (tends to drift with temperature changes).

## 📦 Bill of Materials
*   Raspberry Pi
*   DS3231 or DS1307 Module (I2C)
*   **CR2032 Battery** (Ensure it is inserted!)

## 🔌 Wiring
Connect the module to the I2C pins.

| RTC Pin | Raspberry Pi | Note |
|---------|--------------|-----------------------------------|
| VCC     | 3.3V or 5V   | Check your module specs. |
| GND     | GND          | Ground |
| SDA     | GPIO 2 (SDA) | Data |
| SCL     | GPIO 3 (SCL) | Clock |

---

## 🛠 Software Setup (The "Linux Way")

The best way to use an RTC on Raspberry Pi is to let the Linux kernel handle it.

### 1. Enable via Device Tree
Edit `/boot/config.txt` (or `/boot/firmware/config.txt` on newer OS):
```bash
sudo nano /boot/firmware/config.txt
```
Add the following line to the end:
*   For DS3231: `dtoverlay=i2c-rtc,ds3231`
*   For DS1307: `dtoverlay=i2c-rtc,ds1307`

### 2. Remove the Fake HW Clock
Raspberry Pi OS uses a "fake" clock by default. Disable it:
```bash
sudo apt-get -y remove fake-hwclock
sudo update-rc.d -f fake-hwclock remove
sudo systemctl disable fake-hwclock
```

### 3. Sync and Verify
After rebooting, check if the RTC is recognized:
```bash
sudo hwclock -r
```
If it shows the correct time, you're all set! 

To write the current system time (from internet) to the RTC:
```bash
sudo hwclock -w
```

---

## 🚀 ROS2 Integration

The `rtc_monitor_node` provides visibility into the RTC status and internal temperature (for DS3231).

**Launch the monitor:**
```bash
ros2 launch xpi_comms rtc.launch.py
```

### 📡 Interface
*   `~/internal_temperature` (`sensor_msgs/Temperature`): Internal silicon temperature of the DS3231 (High precision).
*   `~/status` (`std_msgs/String`): Current hardware clock string.

## ⚠️ Troubleshooting
*   **"hwclock: Cannot access the Hardware Clock"**: 
    *   Verify I2C wiring.
    *   Check `i2cdetect -y 1`. You should see `UU` (owned by driver) at address `0x68`.
*   **Time resets to 1970**:
    *   Check the CR2032 battery voltage. 
    *   Ensure the battery is making good contact.
