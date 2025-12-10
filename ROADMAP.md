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
*   **Universal Serial Bridge (UART/USB)** -> `xpi_comms/serial_bridge`
*   **Sonar Trend Analysis with LLM** -> `xpi_llm/sonar_trend_analyzer`
*   **LLM Tool Calling** -> `xpi_llm/tool_calling`
*   **Sonar Pattern Analysis with LLM** -> `xpi_llm/sonar_pattern_analyzer`
*   **DS18B20 (1-Wire Temp Sensor)** -> `xpi_sensors/ds18b20`
*   **GPIO Digital Input** -> `xpi_sensors/gpio_digital_input`
*   **#24 MPU6050 (I2C)** -> `xpi_sensors/mpu6050`
*   **#15 TB6612FNG (GPIO)** -> `xpi_actuators/tb6612_driver`
*   **#9 BME280 (I2C)** -> `xpi_sensors/bme280`
*   **#2 BMP085/180 (I2C)** -> `xpi_sensors/bmp085`
*   **#28 WS2812 (Addressable RGB LED)** -> `xpi_actuators/ws2812_driver`
*   **LED Bar** -> `xpi_actuators/led_bar`
*   **LED Matrix 8x8 (MAX7219)** -> `xpi_actuators/led_matrix`
*   **I2C OLED (SSD1306)** -> `xpi_actuators/ssd1306`
*   **TM1637 (GPIO)** -> `xpi_actuators/tm1637`
*   **Unipolar Stepper (ULN2003)** -> `xpi_actuators/unipolar_stepper`
*   **A4988/DRV8825 Stepper (Step/Dir)** -> `xpi_actuators/a4988_driver`
*   **L298/L293 Stepper/DC Driver** -> `xpi_actuators/l298_driver`
*   **Analog Sensor Interpreter (via ADS1115)** -> `xpi_sensors/analog_sensor_interpreter`
*   **BH1750 (Light Sensor)** -> `xpi_sensors/bh1750_node`
*   **Camera (USB/CSI)** -> `blocks/sensors/camera` (Wrapper for `v4l2_camera`)


## 🚧 High Priority (Next Up)


## 📋 backlog: Supported Sensors & Actuators

### Sensors (Vision)
*   [ ] **Camera (USB/CSI)** - *Implemented via v4l2_camera wrapper*

### Light & Color (I2C)
*   [ ] #10 TCS34725 - RGB Color sensor
*   [ ] #3 OPT3001 - High sensitivity light sensor
*   [ ] **TSL2591** - High Dynamic Range Light Sensor
*   [ ] **LTR-390** - UV Light Sensor
*   [ ] **VEML7700** - High Accuracy Ambient Light Sensor
*   [ ] **AS7341** - 11-Channel Spectral Color Sensor
*   [ ] **APDS-9960** - RGB, Gesture, Proximity
*   [ ] #25 MAX44009 - Light sensor
*   [ ] #26 TSL2561 - Light sensor
*   [ ] #34 BH1745NUC - Light sensor

### Environment (Temp, Press, Hum, Gas) (I2C/SPI)
*   [ ] #6 SHT20/HTU21D - Temp/Humidity
*   **#11 LM75A (I2C)** -> `xpi_sensors/lm75a`
*   [ ] #37 CCS811 - CO2
*   [ ] #39 HDC1080 - Temp/Humidity
*   [ ] #40 LPS22HB - Pressure/Temp
*   [ ] #41 SHTC3 - Temp/Humidity

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
4. LCD1602/1604 (I2C) - via PCF8574
5. 74HC595 (HC595) - Shift register (serial to parallel) (GPIO)
6. 74HC165 (HC165) - Shift register (parallel to serial)  (GPIO)
7. PIR Sensors (HC-SR501, MH-SR602, AM312) - PIR sensor (GPIO)
8. VEML6070 - UV sensor
9. Soil moisture sensor (along with ADS1115) - *Now covered by Analog Sensor Interpreter*
10. Vibration sensor (along with ADS1115) - *Now covered by Analog Sensor Interpreter*
11. Noise level sensor (based on microphone with ADS1115) - *Now covered by Analog Sensor Interpreter*
12. AT24Cxxx - memory chip (I2C)
13. MICS-6814 - CO NO2 NH3 gas sensor
14. DSM501A - PM2.5 sensor
15. DHT22 (DHT11) - humidity and temperature sensor
16. W25Qxx (Flash SPI)
17. HW-MS03 - radar sensor module (human sensor)
18. NAP07 HIS07 - smoke sensor
19. JSN-SR04T - ultrasonic sensor (distance)
20. TCS3200 (GY-31) - color sensor
21. MAX30102 - heartrate sensor
22. HX711 (cell weight sensor) - digital load sensor
23. HR-202 - humidity sensor
24. SGP30 - CO2 sensor
*   **MQ-x Gas Sensors (via Analog Sensor Interpreter)** -> `xpi_sensors/analog_sensor_interpreter` - *Now covered by Analog Sensor Interpreter*
26. BME-680 - environmental sensor
27. ZP-16 - gas sensor
28. VL53L1X - distance laser sensor
29. TOF10120(TOF05140) - distance laser sensor (UART/I2C)
30. FPM10A - fingerprint sensor  (UART)
31. BF350 - load cell/strain gauge
32. HW-526 - rotation sensor
33. MAX471 - current sensor (along with ads1115, I2C) - *Now covered by Analog Sensor Interpreter*
34. KY-013 - thermistor/analog temperature sensor (along with ads1115, I2C) - *Now covered by Analog Sensor Interpreter*
35. TGS2600 - air quality PM10 sensor
36. L298D/L298P - powerful motor driver (2 motors) with integrated circuit sensor. - *Already Implemented*
37. WS2812B (WS2813, APA102, SK6812) RGB leds with serial connection (PWM, SPI)

### 🆕 Proposed Actuators
38. **Buzzer / Speaker** - Passive/Active piezo (GPIO/PWM) for status beeps and RTTTL melodies.
39. **Direct GPIO Servo** - Single servo control via software PWM (gpiozero) without external driver.
40. **PWM Breathing LED** - Single channel LED with smooth fading/pulsing effects.
41. **ESC (Electronic Speed Controller)** - BLDC motor control via PWM (Standard servo protocol).