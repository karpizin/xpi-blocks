# XPI-Blocks: Library Index

This index provides direct links to the documentation for every implemented block in the library.

## 📦 Projects (Integrated Systems)
*   **[Smart Motor Unit](../projects/smart_motor/README.md)** - Closed-loop motor control (Driver + Encoder + Current).
*   **[Weather Station](../projects/weather_station/README.md)** - Integrated environment monitor (BME280 + CCS811 + MAX44009).

## 🛠 Developer Tools
*   **[xpi-top Monitor](../src/xpi_tools/README.md)** - TUI dashboard for viewing sensor data (SSH-friendly).

## 🎙️ Audio & Sound Intelligence
*   **[ReSpeaker Mic Array v2.0](../blocks/sensors/audio/respeaker/README.md)** - 4-mic array with Direction of Arrival (DOA) and voice detection.
*   **[Audio Level Monitor](../blocks/sensors/audio/analyzer/README.md)** - Real-time noise level (dB) and RMS monitoring.
*   **[YAMNet Sound Classifier](../blocks/sensors/audio/yamnet/README.md)** - Local Edge AI classification of 500+ sound types (Siren, Bark, etc.).

## 👁️ Vision & AI (CV / VLM / MCP)
*   **[MCP Agent Node](../blocks/llm/mcp_agent/README.md)** - **The Brain.** Context-aware host for real-time sensor monitoring and tool orchestration (Gemini 3.0 Flash).
*   **[LLM Tool Calling](../blocks/llm/tool_calling/README.md)** - Enabling LLMs to control actuators via ROS2 services.
*   **[VLM Observer](../blocks/llm/vlm_observer/README.md)** - Scene description using Vision-Language Models.
*   **[Camera (USB/CSI)](../blocks/sensors/camera/README.md)** - Standard V4L2 camera setup.
*   **[ArUco Navigation](../blocks/vision/aruco_navigation/README.md)** - Precise visual positioning using fiducial markers.
*   **[Sonar Trend Analysis](../blocks/llm/sonar_trend_analysis/README.md)** - Intelligent obstacle distance analysis.
*   **[Sonar Pattern Analysis](../blocks/llm/sonar_pattern_analysis/README.md)** - Identifying movement patterns.

## 📡 Communication & Interfaces
*   **[MQTT Bridge](../blocks/comms/mqtt_bridge/README.md)** - Bidirectional gateway to Home Assistant / IoT (Smart Gateway with JSON support).
*   **[Raw LoRa (SX1276/SX1278)](../blocks/comms/lora_raw/README.md)** - Low-level long-range SPI radio communication.
*   **[HC-12 Wireless Serial](../blocks/comms/hc12/README.md)** - Long-range 433MHz wireless UART bridge.
*   **[Universal Serial Bridge](../blocks/comms/serial_bridge/README.md)** - Bidirectional UART communication (Arduino/ESP32).
*   **[Modbus RTU Master](../blocks/comms/modbus_rtu/README.md)** - Generic RS-485 reader for industrial sensors.
*   **[LTE / 4G Modem](../blocks/comms/lte_4g/README.md)** - Cellular communication, SMS and GPS (SIM7600).
*   **[Real-Time Clock (RTC)](../blocks/comms/rtc/README.md)** - Accurate timekeeping without internet (DS3231/DS1307).
*   **[BLE Bridge](../blocks/comms/ble_bridge/README.md)** - Smartphone control via Bluetooth Low Energy (GATT).
*   **[CAN Bus (MCP2515)](../blocks/comms/can_bus/README.md)** - SocketCAN bridge for automotive/industrial hardware.

## 🎮 Inputs (Human & RC)
*   **[Joystick / Gamepad](../blocks/inputs/joystick/README.md)** - Bluetooth/USB gamepad (Xbox/PS4) teleop.
*   **[Web Virtual Joystick](../blocks/inputs/web_joystick/README.md)** - Smartphone control via browser.
*   **[Telegram Bot Control](../blocks/inputs/telegram_bot/README.md)** - Remote control via chat interface.
*   **[Gesture Control (MediaPipe)](../blocks/inputs/gesture_control/README.md)** - Hand tracking for robot control.
*   **[Joy Mapper (Universal)](../blocks/inputs/joy_mapper/README.md)** - Map Joystick/RC to robot commands (Twist/Bool).
*   **[Mouse & Touchpad](../blocks/inputs/mouse/README.md)** - Use mouse as a joystick (Velocity/Position control).
*   **[Keyboard Teleop](../blocks/inputs/keyboard/README.md)** - Control robot with keyboard arrows.
*   **[Rotary Encoder (GPIO)](../blocks/inputs/rotary_encoder/README.md)** - Quadrature encoder for dials or motor odometry.
*   **[RC Receiver (SBUS)](../blocks/inputs/rc_sbus/README.md)** - FrSky/Futaba radio integration.
*   **[RC Receiver (CRSF)](../blocks/inputs/rc_crsf/README.md)** - TBS Crossfire / ELRS integration.
*   **[RC Receiver (PPM)](../blocks/inputs/rc_ppm/README.md)** - Legacy PPM radio integration.

## 🌡️ Sensors (Environment & Weather)
*   **[SCD40 / SCD41 CO2 Sensor](../blocks/sensors/environment/scd4x/README.md)** - True NDIR CO2, Temperature, and Humidity (I2C).
*   **[AHT10 / AHT20](../blocks/sensors/environment/aht20/README.md)** - High-precision temperature and humidity (I2C).
*   **[SHT30 / SHT31 / SHT35](../blocks/sensors/environment/sht3x/README.md)** - High-precision temperature and humidity (Sensirion I2C).
*   **[DHT11 / DHT22 Temperature](../blocks/sensors/dht/README.md)** - Low-cost temp/humidity sensor (GPIO).
*   **[BME280 Environment](../blocks/sensors/environment/README.md)** - Temperature, Humidity, Pressure (I2C).
*   **[BME680 Environment](../blocks/sensors/bme680/README.md)** - Gas, Temperature, Humidity, Pressure (I2C).
*   **[BH1750 Light Sensor](../blocks/sensors/bh1750/README.md)** - Ambient light intensity (Lux).
*   **[OPT3001 Light Sensor](../blocks/sensors/opt3001/README.md)** - High-precision ambient light (Lux).
*   **[MAX44009 Light Sensor](../blocks/sensors/max44009/README.md)** - Ultra-wide dynamic range lux meter.
*   **[TSL2561 Light Sensor](../blocks/sensors/tsl2561/README.md)** - Dual-diode wide dynamic range light sensor.
*   **[TSL2591 Light Sensor](../blocks/sensors/tsl2591/README.md)** - High Dynamic Range Light Sensor (I2C).
*   **[VEML6070 UV Sensor](../blocks/sensors/environment/uv_veml6070/README.md)** - Ultraviolet light intensity (I2C).
*   **[TCS34725 Color Sensor](../blocks/sensors/tcs34725/README.md)** - RGB color and light sensor with IR filter.
*   **[TCS3200 Color Sensor](../blocks/sensors/color/tcs3200/README.md)** - Fast color-to-frequency converter (GPIO).
*   **[AS7341 Spectral Sensor](../blocks/sensors/as7341/README.md)** - 11-Channel spectral color analyzer.
*   **[RFID/NFC (PN532)](../blocks/sensors/nfc/README.md)** - Contactless tag identification via I2C.
*   **[Anemometer Wind Speed](../blocks/sensors/anemometer/README.md)** - Mechanical cup anemometer (GPIO).
*   **[Wind Vane (Analog)](../blocks/sensors/environment/wind_vane/README.md)** - Wind direction (16 points, Analog).
*   **[Wind Vane (Digital)](../blocks/sensors/environment/wind_vane_digital/README.md)** - High-precision wind direction (AS5600 I2C).
*   **[Rain Gauge](../blocks/sensors/environment/rain_gauge/README.md)** - Precipitation intensity and total (GPIO).
*   **[PIR Motion Sensor](../blocks/sensors/movement/pir/README.md)** - Basic infrared motion detection (GPIO).
*   **[CCS811 Air Quality](../blocks/sensors/ccs811/README.md)** - eCO2 and TVOC sensor.
*   **[MICS-6814 Triple Gas](../blocks/sensors/mics6814/README.md)** - CO, NO2, and NH3 analog gas sensor.
*   **[PMS5003 Particulate Matter](../blocks/sensors/pms5003/README.md)** - PM1.0, PM2.5, PM10 laser dust sensor.
*   **[BMP085/180 Pressure](../blocks/sensors/environment/README.md)** - Barometric pressure and altitude (I2C).
*   **[DS18B20 Temperature](../blocks/sensors/temperature/README.md)** - Waterproof 1-Wire temperature sensor.
*   **[HR202 Humidity Sensor](../blocks/sensors/hr202/README.md)** - Low-cost analog resistive humidity sensor.
*   **[KY-013 Thermistor](../blocks/sensors/ky013/README.md)** - Analog NTC thermistor module.
*   **[HC-SR04 Sonar](../blocks/sensors/range/README.md)** - Ultrasonic distance measurement (GPIO).
*   **[VL53L1X ToF Sensor](../blocks/sensors/range/vl53l1x/README.md)** - Precision laser distance measurement (I2C).
*   **[MPU6050 IMU](../blocks/sensors/imu/README.md)** - Accelerometer and Gyroscope (6-DOF).
*   **[MPU9250 IMU](../blocks/sensors/imu/mpu9250/README.md)** - 9-DOF sensor (Accel, Gyro, Mag) via I2C.
*   **[BNO055 IMU](../blocks/sensors/imu/bno055/README.md)** - Intelligent 9-DOF orientation with sensor fusion (I2C).
*   **[Analog Input (ADS1115)](../blocks/sensors/analog_input/README.md)** - Reading analog sensors (Soil, Gas, Light, etc.).
*   **[Digital Input](../blocks/sensors/digital_input/README.md)** - Buttons, switches, IR obstacles.

## 🗺️ Navigation & Localization
*   **[Optical Flow (PMW3901)](../blocks/navigation/optical_flow/README.md)** - 2D ground tracking for precise indoor velocity and displacement (SPI).
*   **[GPS/GNSS NMEA](../blocks/sensors/gps_nmea/README.md)** - Standard GPS tracking via UART.
*   **[QMC5883L Compass](../blocks/sensors/magnetometer_qmc5883l/README.md)** - 3-Axis digital magnetometer for heading.
*   **[UWB Beacon SLAM (Indoor)](../blocks/navigation/uwb_beacons/README.md)** - Indoor SLAM using radio beacons.
*   **[BLE Beacon SLAM](../blocks/navigation/ble_slam/README.md)** - Indoor localization using Bluetooth Low Energy RSSI trilateration.
*   **[BLE Beacon Ranging (RSSI)](../blocks/navigation/ble_beacons/README.md)** - Low-cost RSSI-based indoor ranging.

## 🔋 Power & Energy
*   **[INA219 Power Monitor](../blocks/sensors/ina219/README.md)** - Voltage, Current, Power sensor (I2C).
*   **[MAX17048 Battery Gauge](../blocks/sensors/battery_max17048/README.md)** - Accurate LiPo fuel gauge (I2C).
*   **[ACS712 Current Sensor](../blocks/sensors/acs712/README.md)** - Analog Hall-effect current sensor (via ADS1115).
*   **[MAX471 Voltage/Current](../blocks/sensors/max471/README.md)** - Analog V/A sensor (via ADS1115).
*   **[EPEver Tracer MPPT](../blocks/power/epever_mppt/README.md)** - Solar charge controller monitoring (Modbus RS-485).

## 💾 Storage & Memory
*   **[AT24Cxxx EEPROM](../blocks/storage/eeprom/README.md)** - Non-volatile I2C memory for config and state.

## ⚙️ Actuators (Motion & Expansion)
*   **[GPIO Expander (MCP23017)](../blocks/actuators/gpio_expander/mcp23017/README.md)** - 16 additional digital I/O pins via I2C.
*   **[Analog Mux (CD74HC4067)](../blocks/sensors/analog_input/cd74hc4067/README.md)** - 16-channel analog input expander.
*   **[Output Shift Register (74HC595)](../blocks/actuators/shift_registers/74hc595/README.md)** - 8-bit serial-to-parallel expander (GPIO).
*   **[Input Shift Register (74HC165)](../blocks/sensors/digital_input/74hc165/README.md)** - 8-bit parallel-to-serial expander (GPIO).
*   **[DC Motor (TB6612)](../blocks/actuators/motors_dc/README.md)** - Dual DC motor driver control.
*   **[DC/Stepper (L298N)](../blocks/actuators/motors_dc/README.md)** - High power motor driver.
*   **[ESC BLDC Motor](../blocks/actuators/motors_esc/README.md)** - Brushless motor control (Drone/Car).
*   **[Direct GPIO Servo](../blocks/actuators/servos/direct_gpio/README.md)** - Single servo via GPIO (Software PWM).
*   **[Servo Controller (PCA9685)](../blocks/actuators/servos/README.md)** - 16-channel PWM servo driver.
*   **[Stepper (ULN2003)](../blocks/actuators/steppers/README.md)** - Cheap 5V unipolar stepper motor.
*   **[Stepper (A4988/DRV8825)](../blocks/actuators/steppers/README.md)** - Precision bipolar stepper driver.

## 💡 Actuators (Visual & Output)
*   **[Buzzer & Melodies](../blocks/actuators/buzzer/README.md)** - Status beeps and RTTTL music (GPIO/PWM).
*   **[Relay Module](../blocks/actuators/relays/README.md)** - Switching high-voltage/current loads.
*   **[OLED Display (SSD1306)](../blocks/actuators/oled_displays/README.md)** - Small graphical display (I2C).
*   **[TFT SPI Display (ST7789)](../blocks/actuators/displays/tft_spi/README.md)** - Color IPS/TFT display supporting ROS2 Images and layered JSON drawing.
*   **[LCD 1602 Display](../blocks/actuators/lcd1602/README.md)** - Character LCD (16x2) via I2C.
*   **[LED Matrix (MAX7219)](../blocks/actuators/led_displays/README.md)** - 8x8 dot matrix display.
*   **[7-Segment (TM1637)](../blocks/actuators/led_displays/README.md)** - 4-digit numeric display.
*   **[WS2812B Effects](../blocks/actuators/leds/ws2812_effects/README.md)** - Addressable RGB LED strips with 100+ effects.
*   **[LED Bar (MY9221)](../blocks/actuators/leds/README.md)** - 10-segment LED bar graph.

## 🎭 Human-Computer Interaction (HCI)
*   **[Robot Face Expressions](../blocks/hci/robot_face/README.md)** - Procedural animated eyes for any display.