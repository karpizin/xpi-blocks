# XPI-Blocks: Library Index

This index provides direct links to the documentation for every implemented block in the library.

## 📦 Projects (Integrated Systems)
*   **[Smart Motor Unit](../projects/smart_motor/README.md)** - Closed-loop motor control (Driver + Encoder + Current).
*   **[Weather Station](../projects/weather_station/README.md)** - Integrated environment monitor (BME280 + CCS811 + MAX44009).

## 🛠 Developer Tools
*   **[xpi-top Monitor](../src/xpi_tools/README.md)** - TUI dashboard for viewing sensor data (SSH-friendly).

## 👁️ Vision & AI (CV / VLM)
*   **[Camera (USB/CSI)](../blocks/sensors/camera/README.md)** - Standard V4L2 camera setup.
*   **[VLM Observer](../blocks/llm/vlm_observer/README.md)** - Scene description using Gemini/GPT-4o.
*   **[Sonar Trend Analysis](../blocks/llm/sonar_trend_analysis/README.md)** - Detecting approach/retreat using LLM.
*   **[Sonar Pattern Analysis](../blocks/llm/sonar_pattern_analysis/README.md)** - Identifying movement patterns.
*   **[LLM Tool Calling](../blocks/llm/tool_calling/README.md)** - Enabling LLMs to control actuators via ROS2 services.

## 📡 Communication & Interfaces
*   **[Universal Serial Bridge](../blocks/comms/serial_bridge/README.md)** - Bidirectional UART communication (Arduino/ESP32).

## 🎮 Inputs (Human & RC)
*   **[Joystick / Gamepad](../blocks/inputs/joystick/README.md)** - Bluetooth/USB gamepad (Xbox/PS4) teleop.
*   **[Web Virtual Joystick](../blocks/inputs/web_joystick/README.md)** - Smartphone control via browser.
*   **[Telegram Bot Control](../blocks/inputs/telegram_bot/README.md)** - Remote control via chat buttons and photos.
*   **[Gesture Control (MediaPipe)](../blocks/inputs/gesture_control/README.md)** - Hand tracking for robot control.
*   **[Joy Mapper (Universal)](../blocks/inputs/joy_mapper/README.md)** - Map Joystick/RC to robot commands (Twist/Bool).
*   **[Mouse & Touchpad](../blocks/inputs/mouse/README.md)** - Use mouse as a joystick (Velocity/Position control).
*   **[Keyboard Teleop](../blocks/inputs/keyboard/README.md)** - Control robot with keyboard arrows.
*   **[Rotary Encoder (GPIO)](../blocks/inputs/rotary_encoder/README.md)** - Quadrature encoder for dials or motor odometry.
*   **[RC Receiver (SBUS)](../blocks/inputs/rc_sbus/README.md)** - FrSky/Futaba radio integration.
*   **[RC Receiver (CRSF)](../blocks/inputs/rc_crsf/README.md)** - TBS Crossfire / ELRS integration.
*   **[RC Receiver (PPM)](../blocks/inputs/rc_ppm/README.md)** - Legacy PPM radio integration.

## 🌡️ Sensors (Environment & Physics)
*   **[BH1750 Light Sensor](../blocks/sensors/bh1750/README.md)** - Ambient light intensity (Lux).
*   **[OPT3001 Light Sensor](../blocks/sensors/opt3001/README.md)** - High-precision ambient light (Lux).
*   **[MAX44009 Light Sensor](../blocks/sensors/max44009/README.md)** - Ultra-wide dynamic range lux meter.
*   **[TSL2561 Light Sensor](../blocks/sensors/tsl2561/README.md)** - Dual-diode wide dynamic range light sensor.
*   **[TCS34725 Color Sensor](../blocks/sensors/tcs34725/README.md)** - RGB color and light sensor with IR filter.
*   **[AS7341 Spectral Sensor](../blocks/sensors/as7341/README.md)** - 11-Channel spectral color analyzer.
*   **[APDS-9960 Gesture/Color](../blocks/sensors/apds9960/README.md)** - RGB, Proximity, and Gesture control sensor.
*   **[BME280 Environment](../blocks/sensors/environment/README.md)** - Temperature, Humidity, Pressure (I2C).
*   **[CCS811 Air Quality](../blocks/sensors/ccs811/README.md)** - eCO2 and TVOC gas sensor.
*   **[BMP085/180 Pressure](../blocks/sensors/environment/README.md)** - Barometric pressure and altitude (I2C).
*   **[DS18B20 Temperature](../blocks/sensors/temperature/README.md)** - Waterproof 1-Wire temperature sensor.
*   **[KY-013 Thermistor](../blocks/sensors/ky013/README.md)** - Analog NTC thermistor module.
*   **[HC-SR04 Sonar](../blocks/sensors/range/README.md)** - Ultrasonic distance measurement (GPIO).
*   **[MPU6050 IMU](../blocks/sensors/imu/README.md)** - Accelerometer and Gyroscope (6-DOF).
*   **[Analog Input (ADS1115)](../blocks/sensors/analog_input/README.md)** - Reading analog sensors (Soil, Gas, Light, etc.).
*   **[Digital Input](../blocks/sensors/digital_input/README.md)** - Buttons, switches, IR obstacles.

## 🔋 Power & Energy
*   **[INA219 Power Monitor](../blocks/sensors/ina219/README.md)** - Voltage, Current, Power sensor (I2C).
*   **[ACS712 Current Sensor](../blocks/sensors/acs712/README.md)** - Analog Hall-effect current sensor (via ADS1115).
*   **[MAX471 Voltage/Current](../blocks/sensors/max471/README.md)** - Analog V/A sensor (via ADS1115).

## ⚙️ Actuators (Motion)
*   **[DC Motor (TB6612)](../blocks/actuators/motors_dc/README.md)** - Dual DC motor driver control.
*   **[DC/Stepper (L298N)](../blocks/actuators/motors_dc/README.md)** - High power motor driver.
*   **[ESC BLDC Motor](../blocks/actuators/motors_esc/README.md)** - Brushless motor control (Drone/Car).
*   **[Direct GPIO Servo](../blocks/actuators/servos/direct_gpio/README.md)** - Single servo via GPIO (Software PWM).
*   **[Servo Controller (PCA9685)](../blocks/actuators/servos/README.md)** - 16-channel PWM servo driver.
*   **[Stepper (ULN2003)](../blocks/actuators/steppers/README.md)** - Cheap 5V unipolar stepper motor.
*   **[Stepper (A4988/DRV8825)](../blocks/actuators/steppers/README.md)** - Precision bipolar stepper driver.

## 💡 Actuators (Visual & Output)
*   **[Relay Module](../blocks/actuators/relays/README.md)** - Switching high-voltage/current loads.
*   **[OLED Display (SSD1306)](../blocks/actuators/oled_displays/README.md)** - Small graphical display (I2C).
*   **[LCD 1602 Display](../blocks/actuators/lcd1602/README.md)** - Character LCD (16x2) via I2C.
*   **[LED Matrix (MAX7219)](../blocks/actuators/led_displays/README.md)** - 8x8 dot matrix display.
*   **[7-Segment (TM1637)](../blocks/actuators/led_displays/README.md)** - 4-digit numeric display.
*   **[WS2812B Effects](../blocks/actuators/leds/ws2812_effects/README.md)** - Addressable RGB LED strips with 100+ effects.
*   **[LED Bar (MY9221)](../blocks/actuators/leds/README.md)** - 10-segment LED bar graph.
