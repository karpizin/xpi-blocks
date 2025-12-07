# XPI-Blocks: Device Roadmap

Total devices targeted: 80+
Current Focus: Establishing core patterns for GPIO, I2C, and PWM.

## ✅ Implemented
*   **#27 Relay Module (GPIO)** -> `xpi_actuators/relay_node`
*   **#7 PCA9685 (I2C)** -> `xpi_actuators/pca9685_node`
*   **#35 HC-SR04 (GPIO)** -> `xpi_sensors/sonar_node`
*   **Joystick/Gamepad** -> `xpi_inputs/joystick`
*   **Keyboard** -> `xpi_inputs/keyboard`
*   **SBUS Receiver** -> `xpi_inputs/rc_sbus`
*   **CRSF Receiver** -> `xpi_inputs/rc_crsf`
*   **PPM Receiver** -> `xpi_inputs/rc_ppm`
*   **Sonar Trend Analysis with LLM** -> `xpi_llm/sonar_trend_analyzer`
*   **LLM Tool Calling** -> `xpi_llm/tool_calling`
*   **Sonar Pattern Analysis with LLM** -> `xpi_llm/sonar_pattern_analyzer`
*   **DS18B20 (1-Wire Temp Sensor)** -> `xpi_sensors/ds18b20`
*   **GPIO Digital Input** -> `xpi_sensors/gpio_digital_input`
*   **#24 MPU6050 (I2C)** -> `xpi_sensors/mpu6050`
*   **#15 TB6612FNG (GPIO)** -> `xpi_actuators/tb6612_driver`
*   **#9 BME280 (I2C)** -> `xpi_sensors/bme280`

## 🚧 High Priority (Next Up)


## 📋 backlog: Supported Sensors & Actuators

### Light & Color (I2C)
*   [ ] #1 BH1750 - Light sensor
*   [ ] #3 OPT3001 - High sensitivity light sensor
*   [ ] #10 TCS34725 - RGB Color sensor
*   [ ] #25 MAX44009 - Light sensor
*   [ ] #26 TSL2561 - Light sensor
*   [ ] #34 BH1745NUC - Light sensor

### Environment (Temp, Press, Hum, Gas) (I2C/SPI)
*   **#2 BMP085/180 (I2C)** -> `xpi_sensors/bmp085`
*   [ ] #6 SHT20/HTU21D - Temp/Humidity
*   [ ] #11 LM75A - Temp + Thermostat
*   [ ] #37 CCS811 - CO2
*   [ ] #39 HDC1080 - Temp/Humidity
*   [ ] #40 LPS22HB - Pressure/Temp
*   [ ] #41 SHTC3 - Temp/Humidity

### IO Expanders & ADC (I2C)
*   [ ] #4 MCP23017 - 16 GPIO Expander
*   **#5 ADS1115 - ADC chip, 4 channels (I2C)** -> `xpi_sensors/ads1115`
*   [ ] #12 PCF8574 - 8 GPIO Expander

### Motors & Drivers
*   [ ] #7a PCA9685_TB6612 - I2C Motor Driver
*   [ ] #16 DRV8833 - Dual DC Motor (GPIO)
*   [ ] #17 VNH2SP30 - High Power Motor (GPIO)
*   [ ] #28 ULN2003 - Unipolar Stepper
*   [ ] #29 L298/L293 - Bipolar Stepper/DC
*   [ ] #36 L9110H + MCP21017 - I2C Motor Driver
*   [ ] #38 L9110H - Simple DC Driver

### Power & Battery
*   [ ] #8 MAX17040G - LiPo Fuel Gauge
*   [ ] #18 ACS712 - Current Sensor (Analog)

### Miscellaneous
*   [ ] #13 MCP410xx - Digital Potentiometer (SPI)
*   [ ] #14 X9Cxxx - Digital Potentiometer (GPIO)
*   [ ] #23 BMI160 - IMU 6DOF
*   [ ] #30 DS1307 - RTC
*   [ ] #31 DS3231N - RTC
*   [ ] #32 DS1302 - RTC (GPIO)
*   [ ] #33 APDS-9960 - Proximity/Gesture

*   **LED Bar** -> `xpi_actuators/led_bar`
1. MPU9250 (9DOF IMU)
2. BMX055 (9DOF IMU)
3. 74HC595 (Shift Register Out)
4. 74HC165 (Shift Register In)
5. PIR Sensors (HC-SR501, etc.)
6. VEML6070 (UV)
7. Soil Moisture (Analog)
8. Vibration (Analog)
9. Noise Level (Analog)
10. SSD1306 (OLED)
11. AT24Cxxx (EEPROM)
12. MICS-6814 (Gas)
13. DSM501A (PM2.5)
14. DHT22 (DHT11) - humidity and temperature sensor
15. W25Qxx (Flash SPI)
16. HW-MS03 (Radar)
17. Smoke Sensors
18. JSN-SR04T (Ultrasonic Sensor)
19. TCS3200 (Color Sensor)
20. MAX30102 (Heartrate Sensor)
21. HX711 (Load Cell Sensor)
22. HR-202 (Humidity Sensor)
23. SGP30 (CO2 Sensor)
24. MQ-x (Gas Sensor)
25. BME-680 (Environmental Sensor)
26. ZP-16 (Gas Sensor)
*   **#28 WS2812 (Addressable RGB LED)** -> `xpi_actuators/ws2812_driver`
28. VL53L1X (Distance Laser Sensor)
29. TOF10120 (Distance Laser Sensor)
30. FPM10A (Fingerprint Sensor)
31. BF350 (Load Cell/Strain Gauge)
32. HW-526 (Rotation Sensor)
33. MAX471 (Current Sensor)
34. KY-013 (Thermistor/Analog Temp Sensor)
35. TGS2600 (Air Quality PM10 Sensor)
36. L298D/L298P (Powerful Motor Driver)
37. WS2812B (RGB LEDs)
