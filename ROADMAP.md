# XPI-Blocks: Device Roadmap

Total devices targeted: 80+
Current Focus: Establishing core patterns for GPIO, I2C, and PWM.

## 🛠 Compatibility Matrix (OS & Language)

| ROS 2 Distro | Ubuntu Version | Python Version | Status | Recommended |
| :--- | :--- | :--- | :--- | :--- |
| **Humble Hawksbill** | **22.04 LTS** | **3.10** | **LTS (Active)** | ⭐ **Yes (Gold Standard)** |
| **Jazzy Jalisco** | **24.04 LTS** | **3.12** | **LTS (Active)** | **Yes (Cutting Edge)** |

## ✅ Implemented

### Communication & Comms
*   **HC-12 (433MHz Wireless Serial)** -> `xpi_comms/hc12_node`
*   **Raw LoRa (SX1276/SX1278)** -> `xpi_comms/lora_raw_node`
*   **Universal Serial Bridge (UART/USB)** -> `xpi_comms/serial_bridge`
*   **CAN Bus (SocketCAN)** -> `xpi_comms/can_bridge_node`
*   **Meshtastic LoRa Bridge** -> `xpi_comms/meshtastic_bridge_node`
*   **Smart MQTT Gateway** -> `xpi_comms/mqtt_gateway_node`

### Sensors (Environment & Light)
*   **#9 BME280 / BME680 (I2C)** -> `xpi_sensors/bme280`
*   **#2 BMP085/180 (I2C)** -> `xpi_sensors/bmp085`
*   **HTU21D / SHT20 / SHT3x** -> `xpi_sensors/htu21d_node`
*   **AHT10 / AHT20** -> `xpi_sensors/aht20`
*   **BH1750 / TSL2561 / TSL2591** -> `xpi_sensors/bh1750_node`
*   **CCS811 Air Quality** -> `xpi_sensors/ccs811_node`
*   **SCD4x (CO2 NDIR)** -> `xpi_sensors/scd4x_node`
*   **SGP30 Gas Sensor** -> `xpi_sensors/sgp30_node`

### Sensors (Motion & Distance)
*   **#35 HC-SR04 (GPIO)** -> `xpi_sensors/sonar_node`
*   **#24 MPU6050 / MPU9250** -> `xpi_sensors/mpu6050`
*   **BNO055 Intelligent IMU** -> `xpi_sensors/bno055_node`
*   **VL53L1X Time-of-Flight** -> `xpi_sensors/vl53l1x_node`
*   **TF-Luna / TFmini Plus** -> `xpi_sensors/tfmini_plus_node`
*   **LDROBOT LD19 / D300 LiDAR** -> `xpi_sensors/ld19_node`

### Power & Storage
*   **INA219 Power Monitor** -> `xpi_sensors/ina219_node`
*   **MAX17048 Battery Gauge** -> `xpi_sensors/max17048_node`
*   **IP5306 Power Bank IC** -> `xpi_sensors/ip5306_node`
*   **SW6106 Power Bank IC** -> `xpi_sensors/sw6106_node`
*   **AT24Cxxx EEPROM (I2C)** -> `xpi_commons/eeprom_node`
*   **W25Qxx Flash (SPI)** -> `xpi_commons/w25qxx_node`

### Actuators & HCI
*   **#7 PCA9685 (I2C)** -> `xpi_actuators/pca9685_node`
*   **WS2812B (NeoPixel)** -> `xpi_actuators/ws2812_driver`
*   **Status Indicator (USIS)** -> `xpi_actuators/status_indicator_node`
*   **LCD 1602 / OLED SSD1306** -> `xpi_actuators/lcd1602_node`
*   **Stepper Drivers (A4988 / ULN2003)** -> `xpi_actuators/a4988_driver`

### AI & LLM
*   **Tool Calling / Function Calling** -> `xpi_llm/tool_calling`
*   **Audio Pattern Analyzer** -> `xpi_llm/audio_analyzer_node`
*   **Sonar Trend Analysis** -> `xpi_llm/sonar_trend_analyzer`

## 🚧 High Priority (Next Up)
*   **Victron VE.Direct Driver**
*   **MLX90640 Thermal Camera**
*   **RTK GPS integration**

## 📋 Backlog (To-Do)
*   **Voice Output (TTS)**
*   **Smart Gripper logic**
*   **UWB Indoor Positioning**
*   **Facial Expressions (LCD/Matrix)**