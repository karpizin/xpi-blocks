# Universal Serial Bridge

This block provides a generic bridge between ROS2 and serial devices (Arduino, ESP32, Sensors). It supports both text-based (ASCII line) and binary protocols.

## 📦 Bill of Materials
*   Raspberry Pi
*   USB-to-TTL Adapter (CP2102, FTDI) OR direct connection to UART pins.
*   Target Device (Arduino, ESP32, etc.)

## 🔌 Wiring
**USB Connection:**
Connect the target device via USB. It will appear as `/dev/ttyUSB0` or `/dev/ttyACM0`.

**GPIO UART Connection:**
*   **TX** (Pi Pin 8/10) <-> **RX** (Target)
*   **RX** (Pi Pin 10/8) <-> **TX** (Target)
*   **GND** <-> **GND**
*   *Note:* You must enable UART via `sudo raspi-config` -> Interface Options -> Serial Port (Login shell: No, Hardware enabled: Yes).

## 🚀 Quick Start
1.  **Build the workspace:**
    ```bash
    colcon build --packages-select xpi_comms
    source install/setup.bash
    ```
2.  **Launch the bridge (Text Mode):**
    ```bash
    ros2 launch xpi_comms serial_bridge.launch.py port:=/dev/ttyUSB0 baudrate:=115200 mode:=text
    ```

## 📡 Interface
### Topics
| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `~/rx` | `std_msgs/String` (Text) <br> `std_msgs/UInt8MultiArray` (Binary) | Output | Data received from the serial device. |
| `~/tx` | `std_msgs/String` (Text) <br> `std_msgs/UInt8MultiArray` (Binary) | Input | Data to send to the serial device. |

### Parameters
*   `port` (string, default: `/dev/ttyUSB0`): Path to the serial device.
*   `baudrate` (int, default: `115200`): Communication speed.
*   `timeout` (float, default: `1.0`): Read timeout in seconds.
*   `mode` (string, default: `text`):
    *   `text`: Reads `\n` terminated lines. Publishes strings. Appends `\n` to sent strings.
    *   `binary`: Reads/Writes raw byte arrays.

## ✅ Verification
1.  **Arduino Sketch (Example):**
    ```cpp
    void setup() { Serial.begin(115200); } 
    void loop() {
      if (Serial.available()) {
        String data = Serial.readStringUntil('\n');
        Serial.print("Echo: ");
        Serial.println(data);
      }
      Serial.println("Ping");
      delay(1000);
    }
    ```
2.  **Run the node:**
    ```bash
    ros2 launch xpi_comms serial_bridge.launch.py
    ```
3.  **Listen to data:**
    ```bash
    ros2 topic echo /serial_bridge/rx
    ```
4.  **Send data:**
    ```bash
    ros2 topic pub --once /serial_bridge/tx std_msgs/msg/String "data: 'Hello ROS'"
    ```
