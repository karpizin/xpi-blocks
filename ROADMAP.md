# XPI-Blocks: Device Roadmap

Total devices targeted: 80+
Current Focus: Establishing core patterns for GPIO, I2C, and PWM.

## 🛠 Compatibility Matrix (OS & Language)

Для стабильной работы XPI-Blocks рекомендуется придерживаться следующих сочетаний:

| ROS 2 Distro | Ubuntu Version | Python Version | Status | Recommended |
| :--- | :--- | :--- | :--- | :--- |
| **Humble Hawksbill** | **22.04 LTS** | **3.10** | **LTS (Active)** | ⭐ **Yes (Gold Standard)** |
| **Jazzy Jalisco** | **24.04 LTS** | **3.12** | **LTS (Active)** | **Yes (Cutting Edge)** |
| Iron Irwini | 22.04 LTS | 3.10 | EOL (Soon) | No |
| Foxy Fitzroy | 20.04 LTS | 3.8 | EOL | No |

**Примечания:**
*   **ARM64 (aarch64)**: Все пакеты оптимизированы для 64-битной версии Raspberry Pi OS / Ubuntu.
*   **Docker**: Сборка через Docker гарантирует правильное окружение независимо от хостовой ОС.

## ✅ Implemented
*   **HC-12 (433MHz Wireless Serial)** -> `xpi_comms/hc12_node`
*   **Raw LoRa (SX1276/SX1278)** -> `xpi_comms/lora_raw_node`
*   **#27 Relay Module (GPIO)** -> `xpi_actuators/relay_node`
*   **#7 PCA9685 (I2C)** -> `xpi_actuators/pca9685_node`
*   **#35 HC-SR04 (GPIO)** -> `xpi_sensors/sonar_node`
*   **Joystick/Gamepad** -> `xpi_inputs/joystick`
*   **HC-12 (433MHz Wireless Serial)** -> `xpi_comms/hc12_node`
*   **Audio Level Monitor** -> `xpi_sensors/audio_level_node`
*   **YAMNet Audio Classifier (Edge AI)** -> `xpi_sensors/yamnet_node`
*   **Audio Pattern Analyzer (LLM)** -> `xpi_llm/audio_analyzer_node`
*   **Joystick/Gamepad** -> `xpi_inputs/joystick`
*   **Keyboard** -> `xpi_inputs/keyboard`
*   **Rotary Encoder (GPIO)** -> `xpi_inputs/rotary_encoder_node`
*   **SBUS Receiver** -> `xpi_inputs/rc_sbus`
*   **CRSF Receiver** -> `xpi_inputs/rc_crsf`
*   **PPM Receiver** -> `xpi_inputs/rc_ppm`
*   **Universal Serial Bridge (UART/USB)** -> `xpi_comms/serial_bridge`
*   **Sonar Trend Analysis with LLM** -> `xpi_llm/sonar_trend_analyzer`
*   **LLM Tool Calling** -> `xpi_llm/tool_calling`
*   **Sonar Pattern Analysis with LLM** -> `xpi_llm/sonar_pattern_analyzer`
*   **DS18B20 (1-Wire Temp Sensor)** -> `xpi_sensors/ds18b20`
*   **GPIO Digital Input** -> `xpi_sensors/gpio_digital_input`
*   **#24 MPU6050 (I2C)** -> `xpi_sensors/mpu6050`
*   **BNO055 Intelligent IMU** -> `xpi_sensors/bno055_node`
*   **ESC Motor Driver** -> `xpi_actuators/esc_driver`
*   **#15 TB6612FNG / DRV8833 (GPIO)** -> `xpi_actuators/tb6612_driver`
*   **#17 VNH2SP30 High Power Driver** -> `xpi_actuators/vnh2sp30_node`
*   **PCA9685 I2C Motor Driver** -> `xpi_actuators/pca9685_motor_node`
*   **#9 BME280 (I2C)** -> `xpi_sensors/bme280`
*   **HTU21D / SHT20 (I2C)** -> `xpi_sensors/htu21d_node`
*   **AHT10 / AHT20 (I2C)** -> `xpi_sensors/aht20`
*   **#2 BMP085/180 (I2C)** -> `xpi_sensors/bmp085`
*   **WS2812 (Addressable RGB LED)** -> `xpi_actuators/ws2812_driver`
*   **LED Bar** -> `xpi_actuators/led_bar`
*   **LED Matrix 8x8 (MAX7219)** -> `xpi_actuators/led_matrix`
*   **I2C OLED (SSD1306)** -> `xpi_actuators/ssd1306`
*   **TM1637 (GPIO)** -> `xpi_actuators/tm1637`
*   **Status Indicator (USIS)** -> `xpi_actuators/status_indicator_node`
*   **LCD 1602 (I2C)** -> `xpi_actuators/lcd1602_node`
*   **Unipolar Stepper (ULN2003)** -> `xpi_actuators/unipolar_stepper`
*   **A4988/DRV8825 Stepper (Step/Dir)** -> `xpi_actuators/a4988_driver`
*   **L298/L293 Stepper/DC Driver** -> `xpi_actuators/l298_driver`
*   **Analog Sensor Interpreter (via ADS1115)** -> `xpi_sensors/analog_sensor_interpreter`
*   **BH1750 (Light Sensor)** -> `xpi_sensors/bh1750_node`
*   **Camera (USB/CSI)** -> `blocks/sensors/camera` (Wrapper for `v4l2_camera`)
*   **Direct GPIO Servo** -> `xpi_actuators/direct_servo`
*   **#3 OPT3001 (Light Sensor)** -> `xpi_sensors/opt3001_node`
*   **#37 CCS811 (Air Quality)** -> `xpi_sensors/ccs811_node`
*   **#10 TCS34725 (RGB Color)** -> `xpi_sensors/tcs34725_node`
*   **APDS-9960 (RGB/Gesture)** -> `xpi_sensors/apds9960_node`
*   **AS7341 (11-Ch Spectral)** -> `xpi_sensors/as7341_node`
*   **MAX44009 (Wide Range Lux)** -> `xpi_sensors/max44009_node`
*   **INA219 (Power Monitor)** -> `xpi_sensors/ina219_node`
*   **ACS712 (Analog Current)** -> `xpi_sensors/acs712.launch.py` (via interpreter)
*   **MAX471 (Analog Current)** -> `blocks/sensors/max471`
*   **KY-013 (Thermistor)** -> `blocks/sensors/ky013`
*   **#26 TSL2561 (Light Sensor)** -> `xpi_sensors/tsl2561_node`
*   **Joy Mapper (Universal Translator)** -> `xpi_inputs/joy_mapper_node`
*   **Mouse/Touchpad Input** -> `xpi_inputs/mouse_node`
*   **Keyboard-as-Gamepad** -> `xpi_inputs/keyboard_to_joy_node`
*   **RC Interpreter** -> `xpi_inputs/rc_interpreter_node`
*   **Web Virtual Joystick** -> `xpi_inputs/web_joystick_node`
*   **Telegram Bot Control** -> `xpi_inputs/telegram_bot_node`
*   **Gesture Control (MediaPipe)** -> `xpi_inputs/gesture_control_node`
*   **xpi-top (CLI Monitor)** -> `xpi_tools`
*   **Modbus RTU Generic Driver** -> `xpi_comms/modbus_rtu_node`
*   **Smart MQTT Gateway (JSON & Multi-topic)** -> `xpi_comms/mqtt_gateway_node`
*   **CAN Bus (SocketCAN)** -> `xpi_comms/can_bridge_node`
*   **EPEver MPPT Solar Controller** -> `blocks/power/epever_mppt`
*   **MAX17048 Battery Gauge** -> `xpi_sensors/max17048_node`
*   **Facial Expressions (Procedural)** -> `xpi_hci/expression_engine_node`
*   **GPS/GNSS (NMEA UART)** -> `xpi_sensors/gps_node`
*   **QMC5883L Magnetometer** -> `xpi_sensors/qmc5883l_node`
*   **ArUco Visual Markers** -> `xpi_vision/aruco_tracker_node`
*   **UWB Beacon SLAM (Indoor)** -> `xpi_navigation/beacon_slam_node`
*   **BLE Beacon Ranging (RSSI)** -> `xpi_navigation/ble_ranging_node`
*   **Meshtastic LoRa Bridge** -> `xpi_comms/meshtastic_bridge_node`
*   **Swarm Consensus Engine** -> `blocks/swarm/consensus/engine.py`
*   **PMS5003 Dust Sensor (PM1.0/2.5/10)** -> `xpi_sensors/pms5003_node`
*   **BME680 (Gas/Temp/Hum/Press)** -> `xpi_sensors/bme680_node`
*   **TSL2591 (HDR Light)** -> `xpi_sensors/tsl2591_node`
*   **DHT11 / DHT22 (Temp/Hum)** -> `xpi_sensors/dht_node`
*   **HR-202 (Analog Humidity)** -> `blocks/sensors/hr202` (via Analog Interpreter)
*   **MICS-6814 (Triple Gas)** -> `blocks/sensors/mics6814` (via Analog Interpreter)
*   **Anemometer (Wind Speed)** -> `xpi_sensors/anemometer_node`
*   **LTE / 4G Modem (SIM7600)** -> `xpi_comms/lte_modem_node`
*   **BLE Bridge (GATT Server)** -> `xpi_comms/ble_bridge_node`
*   **ST7789 / ST7735 TFT (SPI)** -> `xpi_actuators/tft_display_node`
*   **RFID/NFC (PN532)** -> `xpi_sensors/nfc_reader_node`
*   **ReSpeaker Mic Array v2.0 (USB)** -> `xpi_sensors/respeaker_node`
*   **AT24Cxxx EEPROM (I2C)** -> `xpi_commons/eeprom_node`
*   **W25Qxx (Flash SPI)** -> `xpi_commons/w25qxx_node`
*   **Audio Level Monitor** -> `xpi_sensors/audio_level_node`
*   **YAMNet Audio Classifier (Edge AI)** -> `xpi_sensors/yamnet_node`
*   **Audio Pattern Analyzer (LLM)** -> `xpi_llm/audio_analyzer_node`
*   **AHT10 / AHT20 (I2C)** -> `xpi_sensors/aht20`
*   **Wind Vane (Analog)** -> `xpi_sensors/wind_vane_node`
*   **Wind Vane (Digital - AS5600)** -> `xpi_sensors/wind_vane_digital_node`
*   **Rain Gauge (Pulse)** -> `xpi_sensors/rain_gauge_node`
*   **VEML6070 (UV Light)** -> `xpi_sensors/veml6070_node`
*   **SHT30 / SHT31 / SHT35 (Precision Temp/Hum)** -> `xpi_sensors/sht3x_node`
*   **SCD40 / SCD41 (True CO2 NDIR)** -> `xpi_sensors/scd4x_node`
*   **SGP30 Gas Sensor (TVOC/eCO2)** -> `xpi_sensors/sgp30_node`
*   **VL53L1X Time-of-Flight** -> `xpi_sensors/vl53l1x_node`

## 🚧 High Priority (Next Up)
*   **Victron VE.Direct Driver** - Support for high-end solar controllers.
*   **SW6106 / IP5306 Drivers** - Support for Power Bank HATs.

## 📋 backlog: Supported Sensors & Actuators

### Sensors (Vision)
*   [ ] **Camera (USB/CSI)** - *Implemented via v4l2_camera wrapper*

### Light & Color (I2C)
*   [ ] **TSL2591** - *Implemented*
*   [ ] **LTR-390** - UV Light Sensor
*   [ ] **VEML7700** - High Accuracy Ambient Light Sensor
*   [x] **AS7341** - 11-Channel Spectral Color Sensor -> `xpi_sensors/as7341_node`
*   [x] **APDS-9960** - RGB, Gesture, Proximity -> `xpi_sensors/apds9960_node`
*   [x] #25 **MAX44009** - Light sensor -> `xpi_sensors/max44009_node`
*   [ ] #34 BH1745NUC - Light sensor

### Environment (Temp, Press, Hum, Gas) (I2C/SPI)
*   [x] **SHT30 / SHT31 / SHT35** - High-Precision Temp/Humidity (I2C) -> `xpi_sensors/sht3x_node`
*   [x] **AHT10 / AHT20** - Low-cost modern I2C Temp/Humidity.
*   [ ] **DPS310** - High-Precision Barometric Pressure (I2C).
*   **#11 LM75A (I2C)** -> `xpi_sensors/lm75a`
*   **HDC1080 Temp/Humidity** -> `xpi_sensors/hdc1080_node`
*   [ ] #40 LPS22HB - Pressure/Temp
*   [ ] #41 SHTC3 - Temp/Humidity
*   [x] **SCD30 / SCD40 / SCD41** - True CO2 (NDIR) Sensors -> `xpi_sensors/scd4x_node`

### Outdoor Weather (Mechanical)
*   [x] **Anemometer** - *Implemented*
*   [x] **Wind Vane** - Wind Direction (Analog) -> `xpi_sensors/wind_vane_node`
*   [x] **Wind Vane (Digital)** - Wind Direction (AS5600 I2C) -> `xpi_sensors/wind_vane_digital_node`
*   [x] **Rain Gauge** - Precipitation (Pulse Counter) -> `xpi_sensors/rain_gauge_node`

### IO Expanders, ADC & Bus Management (I2C)
*   [x] **MCP23017 16-Channel GPIO Expander** -> `xpi_actuators/mcp23017_node`
*   [x] **CD74HC4067 16-Channel Analog Mux** -> `xpi_sensors/analog_mux_4067`
*   [x] **TCA9548A 8-Channel I2C Multiplexer** -> `xpi_commons/i2c_mux_node`
*   [x] **MCP4725 DAC** -> `xpi_actuators/mcp4725_node`
*   [ ] **ISO1540 I2C Isolator** - Galvanic isolation for noisy environments.
*   [ ] #5 ADS1115 - 4-CH ADC (Vital for analog sensors) - *Already Implemented*
*   [ ] #12 PCF8574 - 8 GPIO Expander

### Advanced Perception & Thermal
*   [ ] **MLX90640 Thermal Camera** - 32x24 pixel IR array for human detection.
*   [x] **AMG8833 Thermal Camera** - 8x8 IR pixel matrix for heat detection.
*   [ ] **VL53L5CX Multi-zone ToF** - 8x8 distance grid (Micro-LiDAR).
*   [ ] **OpenMV / Pixy2 Bridge** - Integration for smart vision modules.

### Precision Motion & Haptics
*   [ ] **AS5600 Magnetic Encoder** - 12-bit absolute orientation for joints.
*   [ ] **TMC2208/2209 Silent Stepper** - UART-based quiet drivers with stall detection.
*   [ ] **DRV2605L Haptic Driver** - Tactile feedback with 100+ vibration effects.

### Human-Computer Interaction (HMI) & Audio
*   [ ] **Matrix Keypad (4x4)** - Classic 16-button input.
*   [ ] **MAX98357A I2S Amplifier** - Digital audio output for RPi.
*   [ ] **DS3231 High Precision RTC** - For offline data logging.
*   [ ] **Hardware Watchdog (WDT)** - External system recovery module.

### Pro-Level Environment
*   [ ] **Sensirion SPS30** - Industrial-grade particulate matter sensor.
*   [ ] **Water Quality (pH/EC)** - Sensors for hydro-robotics.

### Motors & Drivers
*   [ ] **PCA9685_TB6612 (I2C)** - Motor Driver HAT support.

1. **MPU9250 (9DOF IMU)** -> `xpi_sensors/mpu9250_node`
2. **ICM-20948 (9DOF IMU)** - High-precision successor to MPU9250
3. **LSM6DSOX / LSM6DS33 (6DOF IMU)** - Industrial grade with ML core
4. **BMI270 / BMI160 (6DOF IMU)** - Ultra-low power Bosch sensors
5. **WT901 / JY901 (9DOF IMU)** - Module with integrated Kalman Filter (UART/I2C)
6. **BNO080 / BNO085 (9DOF IMU)** - Enhanced orientation sensor for VR/Robotics
7. BMX055 (9DOF IMU)
8. LED 7-segment (single digit, GPIO)
9. **74HC595 (HC595) Shift Register (Out)** -> `xpi_actuators/shift_register_595`
10. **74HC165 (HC165) Shift Register (In)** -> `xpi_sensors/shift_register_165`
6. **PIR Sensors (HC-SR501, AM312)** -> `xpi_sensors/pir_node`
7. Soil moisture sensor (along with ADS1115) - *Now covered by Analog Sensor Interpreter*
8. Vibration sensor (along with ADS1115) - *Now covered by Analog Sensor Interpreter*
9. Noise level sensor (based on microphone with ADS1115) - *Now covered by Analog Sensor Interpreter*
10. W25Qxx (Flash SPI)
11. HW-MS03 - radar sensor module (human sensor)
*   **RCWL-0516 Doppler Radar** -> `xpi_sensors/rcwl0516_node`
*   **Cliff Sensor (TCRT5000)** - Fall prevention for mobile bases
14. **Vibration Sensor (SW-420)** - Impact and tilt detection
20. **Optical Flow (PMW3901)** -> `xpi_navigation/optical_flow_node`
21. NAP07 HIS07 - smoke sensor
22. JSN-SR04T - ultrasonic sensor (distance)
23. **BNO055 Intelligent IMU** - 9-DOF orientation with onboard sensor fusion
24. **AMG8833 Thermal Camera** - 8x8 IR pixel matrix for heat detection
25. **VL53L5CX Multi-zone ToF** - 8x8 distance grid (Micro-LiDAR)
26. **Hall Effect Speed Sensors** - Internal wheel odometry
27. **Whiskers (Bumper Switches)** - Physical contact detection
28. **TF-Luna / TFmini Plus** -> `xpi_sensors/tfmini_plus_node`
29. **Luxonis OAK-D (Spatial AI Camera)** - Stereo depth and AI acceleration
30. **Intel RealSense L515 (MEMS LiDAR)** - High-resolution point clouds
*   **LDROBOT LD19 / D300** -> `xpi_sensors/ld19_node`
*   **Livox Mid-360** - Industrial high-range 3D LiDAR
33. **CE30-A Solid-State LiDAR** - Wide FoV obstacle avoidance
34. **Garmin Lidar-Lite v3/v4** - High-precision 40m rangefinder
35. TCS3200 (GY-31) - color sensor

> See [**LIDAR_STRATEGY.md**](docs/LIDAR_STRATEGY.md) for more technical details.

24. MAX30102 - heartrate sensor
25. **HX711 (cell weight sensor)** -> `xpi_sensors/hx711_node`
26. SGP30 - CO2 sensor
22. ZP-16 - gas sensor
23. TOF10120(TOF05140) - distance laser sensor (UART/I2C)
24. FPM10A - fingerprint sensor  (UART)
25. BF350 - load cell/strain gauge
26. TGS2600 - air quality PM10 sensor
27. L298D/L298P - powerful motor driver (2 motors) with integrated circuit sensor. - *Already Implemented*
28. **WS2812B Effects Library** -> `xpi_actuators/ws2812_driver` (Updated with 100+ effects)

### 🆕 Proposed Actuators
29. **Buzzer / Speaker** -> `xpi_actuators/buzzer_node`
30. **PWM Breathing LED** - Single channel LED with smooth fading/pulsing effects.

## 🚀 Future Horizons (New Categories)

### 🎙️ Audio & Voice Interaction
*   [x] **Audio Input:** Support for **ReSpeaker** Mic Arrays (Sound source localization).
*   **Audio Output (TTS):** Integration with **Piper** or **Espeak** for local Text-to-Speech.
*   **Voice Assistant Project:** Combining Mic + TTS + LLM for natural interaction.

### 🗺️ Outdoor Navigation
*   **RTK GPS** -> `xpi_sensors/gps_rtk_node`

### 🖐️ Haptics & Manipulation
*   **Tactile Sensing:** **FSR** (Force Sensitive Resistors) for gripper feedback.
*   **Haptic Feedback:** **DRV2605** LRA/ERM driver for tactile alerts.
*   **Smart Gripper:** Integrated servo control with force/position feedback.

### 🔐 Security & Identification
*   **RFID/NFC (PN532)** -> `xpi_sensors/nfc_reader_node`
*   **RC522 RFID Reader** - Low-cost SPI alternative.
*   **Biometrics:** Fingerprint sensor integration (FPM10A).
*   **Indoor Positioning:** **UWB** (Ultra-Wideband) modules for precise local tracking.

### 🏭 Industrial & Reliability
*   **Multi-cell BMS:** I2C/UART monitoring for 3S-6S battery packs.

### 🎭 Human-Computer Interaction (HCI)
*   **Status Beeps:** Standardized RTTTL melody library for system alerts.
