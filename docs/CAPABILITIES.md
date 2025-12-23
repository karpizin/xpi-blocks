# XPI-Blocks: Comprehensive Capabilities Overview

This document provides a technical summary of all implemented modules (blocks) within the XPI-Blocks library. It is designed for quick indexing by developers and Large Language Models.

## 📦 Integrated Systems (Projects)
*   **Smart Motor Unit:** A closed-loop actuator system combining motor drivers (TB6612/L298N) with rotary encoders for odometry and INA219 for real-time load/stall detection.
*   **Integrated Weather Station:** A multi-sensor environmental monitor merging BME280 (Meteo), CCS811 (Air Quality), and MAX44009 (High-range Light) data.

## 🧠 Artificial Intelligence & LLM (xpi_llm)
*   **LLM Tool Calling:** A bridge enabling Large Language Models to interact with the physical world by invoking ROS2 services and actions via natural language.
*   **VLM Observer:** Vision-Language Model integration for semantic scene description and object reasoning using Gemini or GPT-4o.
*   **Sonar Trend & Pattern Analysis:** Time-series analysis of distance data using LLMs to detect complex movement patterns and behavioral anomalies.

## 🎭 Human-Computer Interaction (HCI)
*   **Procedural Robot Face:** An advanced facial expression engine rendering 50+ animated emotions (eyes, brows, mouth) with smooth transitions and auto-blink logic.
*   **Multi-Display Adapter:** Universal rendering logic that scales facial expressions from 16x16 LED matrices to high-resolution HDMI/DSI screens.

## 🗺️ Navigation & Localization
*   **GPS/GNSS NMEA:** Global positioning driver for UART-based modules (u-blox M8N/M9N) publishing standard `NavSatFix` messages.
*   **QMC5883L Compass:** 3-axis digital magnetometer driver providing precise magnetic heading and orientation data.
*   **UWB Beacon SLAM:** Centimeter-level indoor localization using Decawave DWM1000 modules with auto-discovery and beacon coordinate optimization.
*   **BLE RSSI Ranging:** Low-cost indoor distance estimation using Bluetooth Low Energy signal strength and path-loss modeling.
*   **ArUco Visual Markers:** 3D pose estimation and navigation relative to printed fiducial markers with full support for camera calibration.

## 🌡️ Sensors: Environment & Physics
*   **PMS5003 Dust Sensor:** Laser scattering sensor for detecting Particulate Matter (PM1.0, PM2.5, PM10).
*   **RFID/NFC (PN532):** Contactless identification of tags and cards via I2C for docking and security.
*   **BME680 Environmental:** High-end 4-in-1 sensor measuring Gas (VOC), Temperature, Humidity, and Pressure.
*   **TSL2591 HDR Light:** High Dynamic Range light sensor (600M:1 contrast) with separate IR and Full-Spectrum channels.
*   **AS7341 Spectral Sensor:** 11-channel visible light spectrometer for precise color matching and light source analysis.
*   **CCS811 Air Quality:** Digital gas sensor for monitoring eCO2 and TVOC levels with internal warm-up management.
*   **BME280 / BMP085:** Industry-standard sensors for high-accuracy temperature, humidity, and barometric pressure measurement.
*   **MAX44009 / BH1750 / TSL2561:** A suite of light sensors covering ultra-wide dynamic ranges from 0.045 Lux up to 188,000 Lux.
*   **TCS34725 RGB Color:** High-sensitivity color sensor with IR blocking filter for accurate surface color detection.
*   **DHT11 / DHT22:** Support for popular low-cost humidity and temperature sensors via GPIO with software timing correction.
*   **HR-202 Humidity:** Analog resistive humidity sensor support via generalized Analog Interpreter.
*   **MPU6050 IMU:** 6-axis motion tracking (Accelerometer + Gyroscope) for robot balance and tilt sensing.
*   **HC-SR04 / DS18B20:** Classic sensors for ultrasonic ranging and 1-Wire waterproof temperature monitoring.

## 🔋 Power & Energy Management
*   **INA219 Power Monitor:** I2C-based high-side current and voltage monitor for real-time battery health tracking.
*   **MAX17048 Fuel Gauge:** Precise LiPo battery state-of-charge (SoC %) monitor using the ModelGauge algorithm.
*   **ACS712 / MAX471:** Analog Hall-effect and shunt-based current sensors for heavy-load monitoring (up to 30A).
*   **EPEver Tracer MPPT:** Industrial-grade solar charge controller integration via Modbus RS-485 for autonomous energy systems.

## ⚙️ Actuators & Motion Control
*   **TB6612FNG / L298N / DRV8825:** Support for various DC and Stepper motor drivers with PWM and direction control.
*   **ESC BLDC Driver:** Standard PWM interface for Electronic Speed Controllers to drive brushless drone or car motors.
*   **PCA9685 / Direct GPIO Servo:** Flexible servo control via dedicated I2C drivers or direct software PWM from Raspberry Pi pins.
*   **TFT SPI Display (ST7789/ST7735):** Full-color IPS/TFT display support with standard `sensor_msgs/Image` interface and layered JSON drawing commands for telemetry.

## 📡 Communication & Interfaces
*   **CAN Bus (SocketCAN):** High-reliability automotive/industrial communication bridge for MCP2515 SPI controllers.
*   **Modbus RTU Master:** Fully configurable RS-485 engine for reading and writing to any industrial PLC or sensor.
*   **MQTT IoT Bridge:** Bidirectional gateway linking ROS2 topics to MQTT brokers for Home Assistant and cloud integration.
*   **Universal Serial Bridge:** Robust UART/USB communication channel for data exchange with Arduino, ESP32, or custom MCUs.

## 🎮 Input Devices
*   **Multi-Receiver RC (SBUS/CRSF/PPM):** Low-latency integration for FrSky, TBS Crossfire, and ELRS radio control systems.
*   **Joy Mapper Node:** A universal translator that converts any input (Joystick, Keyboard, Gesture) into standard robot `Twist` commands.
*   **Web Virtual Joystick:** A zero-install browser-based gamepad for controlling robots from a smartphone.
*   **Telegram Bot Control:** Remote teleoperation and status monitoring via a secure messaging interface with photo support.

## 🛠 Developer Tools
*   **xpi-top Monitor:** A real-time TUI (Terminal User Interface) dashboard for monitoring all active sensors and topics over SSH.