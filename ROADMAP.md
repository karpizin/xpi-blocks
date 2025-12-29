# XPI-Blocks: Device Roadmap

Total devices targeted: 80+
Current Focus: Establishing core patterns for GPIO, I2C, and PWM.

## 🛠 Compatibility Matrix (OS, ROS2 & Python)

For stable operation of XPI-Blocks, we recommend the following combinations:

| ROS 2 Distro | Ubuntu Version | Python Version | Status | Recommended |
| :--- | :--- | :--- | :--- | :--- |
| **Humble Hawksbill** | **22.04 LTS** | **3.10** | **LTS (Active)** | ⭐ **Yes (Gold Standard)** |
| **Jazzy Jalisco** | **24.04 LTS** | **3.12** | **LTS (Active)** | **Yes (Cutting Edge)** |
| Iron Irwini | 22.04 LTS | 3.10 | EOL (Soon) | No |
| Foxy Fitzroy | 20.04 LTS | 3.8 | EOL | No |

**Notes:**
*   **ARM64 (aarch64)**: All packages are optimized for the 64-bit version of Raspberry Pi OS / Ubuntu.
*   **Docker**: Building via Docker ensures the correct environment regardless of the host OS.

## ✅ Implemented

### Communication & Comms (Wireless & Wired)
...
*   **RTK GPS Integration (u-blox ZED-F9P)** -> `xpi_sensors/gps_rtk_node`
*   **NTRIP Client (RTCM Corrections)** -> `xpi_sensors/ntrip_client_node`

### Sensors (Environment & Light)
*   **Bosch BME280 / BME680 (I2C/SPI)** -> `xpi_sensors/bme280`
*   **Bosch BMP085 / BMP180 / BMP280** -> `xpi_sensors/bmp085`
*   **Sensirion SHT30 / SHT31 / SHT35** -> `xpi_sensors/sht3x_node`
*   **AHT10 / AHT20 (I2C Humidity/Temp)** -> `xpi_sensors/aht20`
*   **BH1750 / TSL2561 / TSL2591 (Lux Meter)** -> `xpi_sensors/bh1750_node`
*   **HTU21D / SHT20 (I2C)** -> `xpi_sensors/htu21d_node`
*   **HDC1080 Temp/Humidity** -> `xpi_sensors/hdc1080_node`
*   **Sciosense CCS811 Air Quality (VOC)** -> `xpi_sensors/ccs811_node`
*   **Sensirion SCD40 / SCD41 (True CO2 NDIR)** -> `xpi_sensors/scd4x_node`
*   **Sensirion SGP30 Gas/eCO2 (I2C)** -> `xpi_sensors/sgp30_node`
*   **PMS5003 Dust Sensor (PM1.0/2.5/10)** -> `xpi_sensors/pms5003_node`
*   **VEML6070 (UV Light)** -> `xpi_sensors/veml6070_node`
*   **Analog Sensor Interpreter (via ADS1115)** -> `xpi_sensors/analog_sensor_interpreter`

### Sensors (Motion, Distance & Lidar)
*   **Ultrasonic HC-SR04 / JSN-SR04T (GPIO)** -> `xpi_sensors/sonar_node`
*   **MPU6050 / MPU9250 (6/9-DOF IMU)** -> `xpi_sensors/mpu6050`
*   **Bosch BNO055 Absolute Orientation** -> `xpi_sensors/bno055_node`
*   **ST VL53L1X Time-of-Flight (ToF)** -> `xpi_sensors/vl53l1x_node`
*   **TF-Luna / TFmini Plus (Lidar Range)** -> `xpi_sensors/tfmini_plus_node`
*   **LDROBOT LD19 / D300 (360 Lidar)** -> `xpi_sensors/ld19_node`
*   **RCWL-0516 Doppler Radar** -> `xpi_sensors/rcwl0516_node`
*   **PIR Motion Sensors (HC-SR501, AM312)** -> `xpi_sensors/pir_node`
*   **HX711 (Load Cell / Weight Sensor)** -> `xpi_sensors/hx711_node`

### Inputs (Human & RC)
*   **Joystick / Gamepad (Standard USB/BT)** -> `xpi_inputs/joystick`
*   **Keyboard-as-Gamepad** -> `xpi_inputs/keyboard`
*   **Rotary Encoder (GPIO)** -> `xpi_inputs/rotary_encoder_node`
*   **RC Receivers (SBUS, CRSF, PPM)** -> `xpi_inputs/rc_sbus`
*   **Telegram Bot Control** -> `xpi_inputs/telegram_bot_node`
*   **Gesture Control (MediaPipe)** -> `xpi_inputs/gesture_control_node`
*   **Joy Mapper (Universal Input Translator)** -> `xpi_inputs/joy_mapper_node`

### Power, Storage & Memory
*   **INA219 Power Monitor (Voltage/Current)** -> `xpi_sensors/ina219_node`
*   **MAX17048 Battery Fuel Gauge** -> `xpi_sensors/max17048_node`
*   **IP5306 Power Bank IC (I2C)** -> `xpi_sensors/ip5306_node`
*   **SW6106 Fast Charge Power Bank IC** -> `xpi_sensors/sw6106_node`
*   **AT24Cxxx EEPROM (I2C)** -> `xpi_commons/eeprom_node`
*   **W25Qxx Flash (SPI)** -> `xpi_commons/w25qxx_node`

### Actuators & HCI
*   **#7 PCA9685 (16-Ch PWM I2C)** -> `xpi_actuators/pca9685_node`
*   **WS2812B (NeoPixel) Effects Library** -> `xpi_actuators/ws2812_driver`
*   **Status Indicator (USIS Standard)** -> `xpi_actuators/status_indicator_node`
*   **LCD 1602 / OLED SSD1306 (I2C)** -> `xpi_actuators/lcd1602_node`
*   **Stepper Drivers (A4988 / ULN2003)** -> `xpi_actuators/a4988_driver`
*   **L298/L293 DC Motor Drivers** -> `xpi_actuators/l298_driver`
*   **Direct GPIO Servo** -> `xpi_actuators/direct_servo`
*   **Buzzer / RTTTL Melodies** -> `xpi_actuators/buzzer_node`

### AI & LLM
*   **Tool Calling / Function Calling** -> `xpi_llm/tool_calling`
*   **Audio Pattern Analyzer (LLM)** -> `xpi_llm/audio_analyzer_node`
*   **Voice Output (Piper TTS)** -> `xpi_audio/piper_tts_node`
*   **Speech-to-Text (Whisper)** -> `xpi_audio/whisper_stt_node`
*   **Sonar Trend Analysis** -> `xpi_llm/sonar_trend_analyzer`
*   **Facial Expressions (Procedural HCI)** -> `xpi_hci/expression_engine_node`

## 🚧 High Priority (Next Up)
*   **Victron VE.Direct Driver** - Support for high-end solar controllers.
*   **MLX90640 Thermal Camera** - 32x24 pixel IR array.
*   **RTK GPS Integration (u-blox ZED-F9P)** - Centimeter-level positioning.
*   **SW6106 / IP5306 Drivers** - *Already implemented, updating docs*

## 📋 Backlog (Future Supported Sensors)
*   **LTR-390** - UV Light Sensor.
*   **VEML7700** - High Accuracy Ambient Light.
*   **DPS310** - High-Precision Barometric Pressure.
*   **SPS30** - Industrial-grade PM sensor.
*   **VL53L5CX Multi-zone ToF** - 8x8 distance grid.
*   **AS5600 Magnetic Encoder** - 12-bit joint positioning.
*   **TMC2209 Silent Steppers** - UART control.
*   **Smart Gripper logic** - Force/position feedback.
*   **UWB Indoor Positioning**.
