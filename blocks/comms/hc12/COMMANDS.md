# HC-12 AT Commands & Configuration Guide

This document provides a detailed reference for configuring the HC-12 module using AT commands through the ROS2 node.

## 🛠 Entering Command Mode
The HC-12 module enters command mode when the **SET** pin is pulled **LOW**. 
In our ROS2 implementation, this is handled automatically when you send a message to the `~/cmd_at` topic.

**Flow:**
1. Node receives a string on `~/cmd_at`.
2. Node pulls GPIO SET to LOW.
3. Node waits 100ms.
4. Node sends the AT command via UART.
5. Node waits 500ms for a response.
6. Node publishes the response to `~/at_response`.
7. Node pulls GPIO SET to HIGH (returns to Transparent Data Mode).

---

## 📋 Common AT Commands

| Command | Description | Default |
| :--- | :--- | :--- |
| `AT` | Test connection. Should return `OK`. | - |
| `AT+V` | Get firmware version. | - |
| `AT+RX` | Retrieve all current parameters (Mode, Channel, Baudrate, Power). | - |
| `AT+Bxxxx` | Set baud rate (e.g., `AT+B9600`, `AT+B115200`). | `9600` |
| `AT+Cxxx` | Set communication channel from 001 to 100 (433.4MHz to 473.0MHz). | `001` |
| `AT+P8` | Set transmit power (1 to 8, where 8 is max +20dBm). | `8` |
| `AT+FUx` | Set communication mode (FU1, FU2, FU3, FU4). | `FU3` |
| `AT+DEFAULT` | Reset all parameters to factory defaults. | - |
| `AT+SLEEP` | Enter low-power sleep mode. | - |

---

## 📡 Communication Modes (FUx)

The mode significantly affects range, latency, and power consumption. **Both modules must be in the same mode.**

### FU1 (Moderate Power Saving)
*   **Idle current:** ~3.6mA.
*   **Latency:** Low (~15-25ms).
*   **Use case:** General purpose, battery-sensitive but needs responsiveness.

### FU2 (Extreme Power Saving)
*   **Idle current:** ~80uA (extremely low).
*   **Baud rate restriction:** Only 1200, 2400, 4800 bps.
*   **Latency:** High (~500ms).
*   **Use case:** Remote sensors sending data every few minutes.

### FU3 (Standard / Full Speed) - DEFAULT
*   **Idle current:** ~16mA.
*   **Latency:** Variable (depends on baud rate, ~4-30ms).
*   **Range:** Up to 600m-1000m.
*   **Use case:** Real-time telemetry, default choice.

### FU4 (Long Range)
*   **Idle current:** ~16mA.
*   **Speed:** Fixed at 1200 bps (over the air).
*   **Range:** Up to 1800m.
*   **Latency:** Very high (~1000ms).
*   **Use case:** Ultra-long distance communication with very small data packets.

---

## 💻 ROS2 Examples

### 1. Check current settings
```bash
# In terminal 1:
ros2 topic echo /hc12/hc12/at_response

# In terminal 2:
ros2 topic pub --once /hc12/hc12/cmd_at std_msgs/String "data: 'AT+RX'"
```

### 2. Change channel to 005
```bash
ros2 topic pub --once /hc12/hc12/cmd_at std_msgs/String "data: 'AT+C005'"
```

### 3. Change baud rate to 115200
> **Note:** After this command, you must restart the node with the new `baudrate` parameter.
```bash
ros2 topic pub --once /hc12/hc12/cmd_at std_msgs/String "data: 'AT+B115200'"
```

### 4. Set to Long Range mode (FU4)
```bash
ros2 topic pub --once /hc12/hc12/cmd_at std_msgs/String "data: 'AT+FU4'"
```
