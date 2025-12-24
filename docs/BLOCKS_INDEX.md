# XPI-Blocks: Library Index

This index provides direct links to the documentation for every implemented block in the library.

## 🎙️ Audio & Sound Intelligence
*   **[ReSpeaker Mic Array v2.0](../blocks/sensors/audio/respeaker/README.md)** - 4-mic array with Direction of Arrival (DOA) and voice detection.
*   **[Audio Level Monitor](../blocks/sensors/audio/analyzer/README.md)** - Real-time noise level (dB) and RMS monitoring.
*   **[YAMNet Sound Classifier](../blocks/sensors/audio/yamnet/README.md)** - Local Edge AI classification of 500+ sound types (Siren, Bark, etc.).

## 👁️ Vision & AI (CV / VLM)
*   **[Camera (USB/CSI)](../blocks/sensors/camera/README.md)** - Standard V4L2 camera setup.
*   **[ArUco Navigation](../blocks/vision/aruco_navigation/README.md)** - Precise visual positioning using fiducial markers.
*   **[VLM Observer](../blocks/llm/vlm_observer/README.md)** - Scene description using Gemini 3.0 Flash.
*   **[Sonar Trend Analysis](../blocks/llm/sonar_trend_analysis/README.md)** - Intelligent obstacle distance analysis.
*   **[LLM Tool Calling](../blocks/llm/tool_calling/README.md)** - High-level robot control via natural language.

## 🌡️ Sensors (Environment & Weather)
*   **[AHT10 / AHT20](../blocks/sensors/environment/aht20/README.md)** - High-precision temperature and humidity.
*   **[BME280 / BME680](../blocks/sensors/environment/README.md)** - Full environmental monitoring (Temp, Hum, Press, Gas).
*   **[Anemometer](../blocks/sensors/anemometer/README.md)** - Wind speed measurement.
*   **[Wind Vane](../blocks/sensors/environment/wind_vane/README.md)** - Wind direction (16 points).
*   **[Rain Gauge](../blocks/sensors/environment/rain_gauge/README.md)** - Precipitation (Precipitation intensity and total).
*   **[CCS811 Air Quality](../blocks/sensors/ccs811/README.md)** - CO2 and TVOC sensor.
*   **[PMS5003 Dust Sensor](../blocks/sensors/pms5003/README.md)** - Laser particulate matter sensor.
*   **[DS18B20 Temperature](../blocks/sensors/temperature/README.md)** - Waterproof 1-Wire sensor.

## 🎮 Human-Computer Interaction (HCI)
*   **[Robot Face Expressions](../blocks/hci/robot_face/README.md)** - Animated eyes and expressions for displays.
*   **[Telegram Bot Control](../blocks/inputs/telegram_bot/README.md)** - Remote control via chat interface.
*   **[Web Virtual Joystick](../blocks/inputs/web_joystick/README.md)** - Browser-based gamepad.

## 📡 Communication
*   **[HC-12 Wireless Serial](../blocks/comms/hc12/README.md)** - Long-range 433MHz UART bridge.
*   **[LTE / 4G Modem](../blocks/comms/lte_4g/README.md)** - SIM7600 integration.
*   **[CAN Bus](../blocks/comms/can_bus/README.md)** - SocketCAN bridge.
*   **[MQTT Gateway](../blocks/comms/mqtt_bridge/README.md)** - IoT connectivity.

## 🗺️ Navigation & Localization
*   **[GPS/GNSS](../blocks/sensors/gps_nmea/README.md)** - Standard UART NMEA tracking.
*   **[QMC5883L Compass](../blocks/sensors/magnetometer_qmc5883l/README.md)** - 3-Axis digital magnetometer.
*   **[UWB Beacon SLAM](../blocks/navigation/uwb_beacons/README.md)** - High-precision indoor tracking.
*   **[BLE Slam](../blocks/navigation/ble_slam/README.md)** - Low-cost Bluetooth-based localization.

## ⚙️ Actuators
*   **[WS2812B Effects](../blocks/actuators/leds/ws2812_effects/README.md)** - Addressable RGB library.
*   **[TFT SPI Display](../blocks/actuators/displays/tft_spi/README.md)** - Advanced color display with JSON drawing support.
*   **[PCA9685 Servos](../blocks/actuators/servos/README.md)** - Multi-channel PWM control.
*   **[Stepper Drivers](../blocks/actuators/steppers/README.md)** - A4988, DRV8825, and ULN2003.
*   **[Motor Controllers](../blocks/actuators/motors_dc/README.md)** - TB6612FNG and L298N.