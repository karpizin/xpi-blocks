# XPI-Blocks: Device Roadmap

Total devices targeted: 80+
Current Focus: Establishing core patterns for GPIO, I2C, and PWM.

## ✅ Implemented
*   **#27 Relay Module (GPIO)** -> `xpi_actuators/relay_node`
*   **#7 PCA9685 (I2C)** -> `xpi_actuators/pca9685_node`
*   **#35 HC-SR04 (GPIO)** -> `xpi_sensors/sonar_node`
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
*   **ESC Motor Driver** -> `xpi_actuators/esc_driver`
*   **#15 TB6612FNG (GPIO)** -> `xpi_actuators/tb6612_driver`
*   **#9 BME280 (I2C)** -> `xpi_sensors/bme280`
*   **#2 BMP085/180 (I2C)** -> `xpi_sensors/bmp085`
*   **WS2812 (Addressable RGB LED)** -> `xpi_actuators/ws2812_driver`
*   **LED Bar** -> `xpi_actuators/led_bar`
*   **LED Matrix 8x8 (MAX7219)** -> `xpi_actuators/led_matrix`
*   **I2C OLED (SSD1306)** -> `xpi_actuators/ssd1306`
*   **TM1637 (GPIO)** -> `xpi_actuators/tm1637`
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
*   **Web Virtual Joystick** -> `xpi_inputs/web_joystick_node`
*   **Telegram Bot Control** -> `xpi_inputs/telegram_bot_node`
*   **Gesture Control (MediaPipe)** -> `xpi_inputs/gesture_control_node`
*   **xpi-top (CLI Monitor)** -> `xpi_tools`
*   **Modbus RTU Generic Driver** -> `xpi_comms/modbus_rtu_node`
*   **MQTT Bridge** -> `xpi_comms/mqtt_bridge_node`
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

## 🚧 High Priority (Next Up)
*   **SCD40 / SCD41** - True CO2 Sensor (NDIR) for accurate indoor air quality.
*   **SGP30 Gas Sensor** - For reliable TVOC/eCO2 readings.
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
*   [ ] #6 SHT20/HTU21D - Temp/Humidity
*   [ ] **SHT30 / SHT31 / SHT35** - Modern High-Precision Temp/Humidity (I2C).
*   [ ] **AHT10 / AHT20** - Low-cost modern I2C Temp/Humidity.
*   [ ] **DPS310** - High-Precision Barometric Pressure (I2C).
*   **#11 LM75A (I2C)** -> `xpi_sensors/lm75a`
*   [ ] #39 HDC1080 - Temp/Humidity
*   [ ] #40 LPS22HB - Pressure/Temp
*   [ ] #41 SHTC3 - Temp/Humidity
*   [ ] **SCD30 / SCD40 / SCD41** - True CO2 (NDIR) Sensors.

### Outdoor Weather (Mechanical)
*   [ ] **Anemometer** - Wind Speed (Pulse Counter / GPIO).
*   [ ] **Wind Vane** - Wind Direction (Analog / Encoder / Gray Code).
*   [ ] **Rain Gauge** - Precipitation (Tipping Bucket / Pulse Counter).

### IO Expanders & ADC (I2C)
*   [ ] #4 MCP23017 - 16 GPIO Expander
*   [ ] #5 ADS1115 - 4-CH ADC (Vital for analog sensors) - *Already Implemented*
*   [ ] #12 PCF8574 - 8 GPIO Expander

### Motors & Drivers
*   [ ] #7a PCA9685_TB6612 - I2C Motor Driver
*   [ ] #16 DRV8833 - Dual DC Motor (GPIO)
*   [ ] #17 VNH2SP30 - High Power Motor (GPIO)

## 🔮 TODO List (Planned)
1. MPU9250 (9DOF IMU)
2. BMX055 (9DOF IMU)
3. LED 7-segment (single digit, GPIO)
4. 74HC595 (HC595) - Shift register (serial to parallel) (GPIO)
5. 74HC165 (HC165) - Shift register (parallel to serial)  (GPIO)
7. PIR Sensors (HC-SR501, MH-SR602, AM312) - PIR sensor (GPIO)
8. VEML6070 - UV sensor
9. Soil moisture sensor (along with ADS1115) - *Now covered by Analog Sensor Interpreter*
10. Vibration sensor (along with ADS1115) - *Now covered by Analog Sensor Interpreter*
11. Noise level sensor (based on microphone with ADS1115) - *Now covered by Analog Sensor Interpreter*
12. AT24Cxxx - memory chip (I2C)
13. MICS-6814 - CO NO2 NH3 gas sensor
14. DSM501A - *Replaced by PMS5003 implementation*
15. DHT22 (DHT11) - *Implemented*
16. W25Qxx (Flash SPI)
17. HW-MS03 - radar sensor module (human sensor)
18. NAP07 HIS07 - smoke sensor
19. JSN-SR04T - ultrasonic sensor (distance)
20. TCS3200 (GY-31) - color sensor
21. MAX30102 - heartrate sensor
22. HX711 (cell weight sensor) - digital load sensor
23. HR-202 - *Implemented via Analog Interpreter*
24. SGP30 - CO2 sensor
*   **MQ-x Gas Sensors (via Analog Sensor Interpreter)** -> `xpi_sensors/analog_sensor_interpreter` - *Now covered by Analog Sensor Interpreter*
26. BME-680 - *Implemented*
27. ZP-16 - gas sensor
28. VL53L1X - distance laser sensor
29. TOF10120(TOF05140) - distance laser sensor (UART/I2C)
30. FPM10A - fingerprint sensor  (UART)
31. BF350 - load cell/strain gauge
33. TGS2600 - air quality PM10 sensor
36. L298D/L298P - powerful motor driver (2 motors) with integrated circuit sensor. - *Already Implemented*
37. **WS2812B Effects Library** -> `xpi_actuators/ws2812_driver` (Updated with 100+ effects)

### 🆕 Proposed Actuators
38. **Buzzer / Speaker** - Passive/Active piezo (GPIO/PWM) for status beeps and RTTTL melodies.
39. [Implemented] **Direct GPIO Servo** - Single servo control via software PWM (gpiozero) without external driver.
40. **PWM Breathing LED** - Single channel LED with smooth fading/pulsing effects.
41. [Implemented] **ESC (Electronic Speed Controller)** - BLDC motor control via PWM (Standard servo protocol).

## 🚀 Future Horizons (New Categories)

### 🎙️ Audio & Voice Interaction
*   **Audio Input:** Support for **ReSpeaker** Mic Arrays (Sound source localization).
*   **Audio Output (TTS):** Integration with **Piper** or **Espeak** for local Text-to-Speech.
*   **Voice Assistant Project:** Combining Mic + TTS + LLM for natural interaction.

### 🗺️ Outdoor Navigation
*   **RTK GPS:** Precision positioning (cm-level) for outdoor rovers.

### 🖐️ Haptics & Manipulation
*   **Tactile Sensing:** **FSR** (Force Sensitive Resistors) for gripper feedback.
*   **Haptic Feedback:** **DRV2605** LRA/ERM driver for tactile alerts.
*   **Smart Gripper:** Integrated servo control with force/position feedback.

### 🔐 Security & Identification
*   **RFID/NFC:** Support for **RC522** and **PN532** readers.
*   **Biometrics:** Fingerprint sensor integration (FPM10A).
*   **Indoor Positioning:** **UWB** (Ultra-Wideband) modules for precise local tracking.

### 🏭 Industrial & Reliability
*   **Multi-cell BMS:** I2C/UART monitoring for 3S-6S battery packs.

### 🎭 Human-Computer Interaction (HCI)
*   **Status Beeps:** Standardized RTTTL melody library for system alerts.