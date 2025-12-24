# MQTT Gateway: Advanced IoT Integration

This block provides a smart gateway between **ROS2** and an **MQTT Broker**. It automatically packages multiple ROS topics into JSON messages and listens for remote MQTT commands.

## 📌 Features
*   **Multi-topic Forwarding:** Map a list of ROS topics to MQTT via YAML config.
*   **JSON Packaging:** Data is sent as structured JSON including timestamps.
*   **Custom Prefixing:** Organize your MQTT namespace (e.g. `robot/sensors/...`).
*   **Bidirectional:** Command the robot from MQTT via the `~/cmd` topic.

## 🚀 Quick Start

1.  **Configure your topics:**
    Edit `src/xpi_comms/config/mqtt_gateway_params.yaml` to include the topics you want to monitor.

2.  **Launch the Gateway:**
    ```bash
    ros2 run xpi_comms mqtt_gateway_node --ros-args --params-file src/xpi_comms/config/mqtt_gateway_params.yaml
    ```

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `broker_address` | string | `localhost` | MQTT Broker IP. |
| `topics_to_mqtt` | list | `[]` | List of ROS topics to push to MQTT. |
| `mqtt_prefix` | string | `xpi_robot/` | Prefix added to all MQTT topics. |

## 📐 Example MQTT Output
Topic: `xpi_robot/scd4x/co2`
```json
{
  "source": "ros2",
  "topic": "/scd4x/co2",
  "value": 450,
  "timestamp": 1703345120.5
}
```