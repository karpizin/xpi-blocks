# Project: Integrated Weather Station

This project builds a comprehensive environmental monitoring station.
It combines standard meteorological data (Temp/Press/Hum) with advanced air quality (eCO2/TVOC) and light level metrics.

## 🧩 Components
1.  **BME280** (I2C 0x76).
2.  **CCS811** (I2C 0x5A).
3.  **MAX44009** (I2C 0x4A).
4.  **Raspberry Pi**.

## ⚡ Wiring
All sensors sit on the same **I2C Bus**.
*   **SDA/SCL** lines connected in parallel.
*   **VCC** -> 3.3V (Check your modules! Some require 5V input if they have onboard LDOs).
*   **CCS811 WAKE** -> GND.

## 🚀 Usage
```bash
ros2 launch xpi_projects weather_station.launch.py
```

## 📡 Output Data
*   `/weather/temp_hum_press` (BME280)
*   `/weather/air_quality` (CCS811)
*   `/weather/light` (MAX44009)
