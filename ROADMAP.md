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
*   **#2 BMP085/180 (I2C)** -> `xpi_sensors/bmp085`
*   **#28 WS2812 (Addressable RGB LED)** -> `xpi_actuators/ws2812_driver`
*   **LED Bar** -> `xpi_actuators/led_bar`
*   **LED Matrix 8x8 (MAX7219)** -> `xpi_actuators/led_matrix`
*   **I2C OLED (SSD1306)** -> `xpi_actuators/ssd1306`
*   **TM1637 (GPIO)** -> `xpi_actuators/tm1637`
*   **Unipolar Stepper (ULN2003)** -> `xpi_actuators/unipolar_stepper`
*   **A4988/DRV8825 Stepper (Step/Dir)** -> `xpi_actuators/a4988_driver`
*   **L298/L293 Stepper/DC Driver** -> `xpi_actuators/l298_driver`


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
*   [ ] #6 SHT20/HTU21D - Temp/Humidity
*   [ ] #11 LM75A - Temp + Thermostat
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

*   **Analog Sensor Interpreter (via ADS1115)** -> `xpi_sensors/analog_sensor_interpreter`