# MQTT Bridge: IoT Gateway

This block creates a bidirectional bridge between **ROS2** and an **MQTT Broker** (like Mosquitto, HiveMQ, or AWS IoT).
It allows your robot to integrate with Home Assistant, Node-RED, or cloud dashboards.

## 🚀 Usage

### 1. Launch
```bash
ros2 launch xpi_comms mqtt_bridge.launch.py
```

### 2. ROS2 to MQTT (Publish)
Send a message to the bridge, and it appears on MQTT.
```bash
ros2 topic pub /mqtt_bridge/tx std_msgs/msg/String "data: 'Hello IoT'" -1
```
*Result:* MQTT topic `robot/tx` receives payload `"Hello IoT"`.

### 3. MQTT to ROS2 (Subscribe)
Send an MQTT message to `robot/rx`. It appears in ROS2.
```bash
mosquitto_pub -t "robot/rx" -m "Stop!"
```
*Result:* ROS2 topic `/mqtt_bridge/rx` receives `"Stop!"`.

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `broker_address` | string | `localhost` | MQTT Broker IP. |
| `broker_port` | int | `1883` | MQTT Port. |
| `username` | string | `""` | Auth User (Optional). |
| `password` | string | `""` | Auth Pass (Optional). |
| `mqtt_topic_sub` | string | `robot/rx` | MQTT topic to listen to. |
| `mqtt_topic_pub` | string | `robot/tx` | MQTT topic to publish to. |

## 🧩 Topics Interface

### Subscribers
*   `~/tx` (`std_msgs/msg/String`) - Messages sent here go to MQTT.

### Publishers
*   `~/rx` (`std_msgs/msg/String`) - Messages received from MQTT appear here.
