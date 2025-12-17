# Web Virtual Joystick

The `web_joystick_node` provides a browser-based dual-stick controller for your robot. It hosts a lightweight web server directly on the Raspberry Pi, allowing you to control the robot from any smartphone, tablet, or laptop connected to the same Wi-Fi network.

![Web Joystick UI](https://i.imgur.com/your-placeholder-image.png)
*(Imagine a dark UI with two virtual thumbsticks)*

## 🎯 Features
*   **Zero Install:** No apps required on the phone. Just open a URL.
*   **Multi-Touch:** Supports dual virtual joysticks (Left: Throttle, Right: Steering).
*   **Real-time:** Low-latency control via WebSockets.
*   **Responsive:** Works on mobile and desktop.

## 🚀 User Guide

### 1. Launch the Node
Run the launch file on your robot:
```bash
ros2 launch xpi_inputs web_joystick.launch.py
```

### 2. Find your Robot's IP
If you don't know the IP address, run:
```bash
hostname -I
```
*Example: `192.168.1.105`*

### 3. Connect
1.  Take your smartphone.
2.  Connect to the same Wi-Fi network as the robot.
3.  Open Chrome or Safari.
4.  Navigate to: `http://<ROBOT_IP>:8080`
    *   *Example:* `http://192.168.1.105:8080`

### 4. Drive!
*   **Left Zone:** Controls Forward/Backward speed (Throttle).
*   **Right Zone:** Controls Turning (Steering).
*   The status at the top should say **"Connected ✅"**.

---

## ⚙️ Configuration

You can configure parameters in the launch file or via CLI.

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `port` | `8080` | Web server port. |
| `scale_linear` | `1.0` | Max linear speed (m/s) for `cmd_vel`. |
| `scale_angular` | `1.5` | Max angular speed (rad/s) for `cmd_vel`. |

**Example (Changing port):**
```bash
ros2 run xpi_inputs web_joystick_node --ros-args -p port:=9090
```

---

## 🛠 Developer Guide

This section explains how the node works and how to modify it.

### Architecture
The node runs a **FastAPI** application inside a separate thread alongside the standard ROS2 executor loop.
*   **Main Thread:** ROS2 Spin loop.
*   **Thread 2:** Uvicorn (ASGI Server) hosting FastAPI.

### Protocol (WebSocket)
The frontend sends JSON payloads to `ws://<HOST>:<PORT>/ws` at ~20Hz.

**Format:**
```json
{
  "lx": 0.5,  // Linear X (Left Stick Vertical): -1.0 to 1.0
  "az": -0.8  // Angular Z (Right Stick Horizontal): -1.0 to 1.0
}
```

### Integration
The node publishes to **two** topics simultaneously:

1.  **`/cmd_vel`** (`geometry_msgs/Twist`)
    *   Direct control. Scaled by `scale_linear` and `scale_angular`.
    *   Useful for simple differential drive robots.

2.  **`/joy`** (`sensor_msgs/Joy`)
    *   Raw normalized input (-1.0 to 1.0).
    *   **Axis 1:** Left Stick Vertical (Linear X).
    *   **Axis 3:** Right Stick Horizontal (Angular Z, inverted to match standard).
    *   Useful for feeding into `joy_mapper_node` for complex logic (e.g., servo control).

### Control your Robot (via Joy Mapper)
To use the Web Joystick as a standard controller for your robot (Arcade Drive), run the mapper with the web-specific configuration:

```bash
ros2 run xpi_inputs joy_mapper_node --ros-args --params-file src/xpi_inputs/config/mapper_web.yaml
```

This configuration maps:
*   **Left Stick (Vertical):** Forward/Backward Speed.
*   **Right Stick (Horizontal):** Turning Speed.

### Customizing the UI
The HTML/JS code is located in:
`src/xpi_inputs/web_static/index.html`

It uses [Nipple.js](https://yoannmoi.net/nipplejs/) for the touch interface. To modify colors, size, or layout, edit the `index.html` file and rebuild the package:
```bash
colcon build --packages-select xpi_inputs
source install/setup.bash
```
*Note: The `index.html` is installed into the `share/xpi_inputs/web_static` directory during build.*
