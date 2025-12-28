# Status Indicator (USIS Manager)

This node implements the **Universal Status Indication System (USIS)**. It acts as a high-level manager that translates system states into specific LED patterns and colors.

## 🚀 How to Use
1.  Launch the WS2812 driver.
2.  Launch the Status Indicator:
    ```bash
    ros2 run xpi_actuators status_indicator_node
    ```
3.  Set a system status:
    ```bash
    ros2 topic pub /status/set std_msgs/String "{data: 'LOW_BATTERY'}"
    ros2 topic pub /status/set std_msgs/String "{data: 'ERROR_CRITICAL'}"
    ```

## 📋 Supported Statuses
| Status String | Visual Output | Meaning |
| :--- | :--- | :--- |
| `SYSTEM_OK` | Green Solid | All systems normal. |
| `BOOT` | White Blink | System is initializing. |
| `LOW_BATTERY` | Orange Double Blink | Battery is below threshold. |
| `ERROR_CRITICAL` | Red SOS | Hardware or software failure. |
| `AI_THINKING` | Purple Breathe | Processing AI/VLM data. |
| `CONNECTED` | Blue Pulse | Connection established. |

## 📡 Interface
### Topics
| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/status/set` | `std_msgs/String` | Input | Set the current system status. |
