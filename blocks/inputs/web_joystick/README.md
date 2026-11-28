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
By default the server listens on `127.0.0.1` (the robot itself). To drive from a phone, let it
listen on the network:
```bash
ros2 launch xpi_inputs web_joystick.launch.py host:=0.0.0.0
```
The node prints the link to open, with a random access token generated at startup:
```
[web_joy]: Web Joystick: open http://<ROBOT_IP>:8080/#token=Xk3...q9w
```
For a token that stays the same across restarts, set it in the environment before launching
(it is not a parameter on purpose: parameters show in `ros2 param get` and `ps`):
```bash
export XPI_WEB_JOYSTICK_TOKEN="$(openssl rand -base64 18)"
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
4.  Open the link from the log, with your robot's IP and the token:
    *   *Example:* `http://192.168.1.105:8080/#token=Xk3...q9w`
    *   Bookmark it: the token is in the `#fragment`, which the browser never sends to servers.

### 4. Drive!
*   **Left Zone:** Controls Forward/Backward speed (Throttle).
*   **Right Zone:** Controls Turning (Steering).
*   The status at the top should say **"Connected ✅"**.

---

## ⚙️ Configuration

You can configure parameters in the launch file or via CLI.

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `host` | `127.0.0.1` | Address to listen on. `0.0.0.0` = all interfaces (needed for a phone). |
| `port` | `8080` | Web server port. |
| `token` | `""` | Access token. Empty: random token at startup, printed to the log. `XPI_WEB_JOYSTICK_TOKEN` overrides it. |
| `allowed_origins` | `""` | Comma-separated extra browser origins allowed to connect (e.g. `http://192.168.1.10:3000`). The page served by the node is always allowed. |
| `scale_linear` | `1.0` | Max linear speed (m/s) for `cmd_vel`. |
| `scale_angular` | `1.5` | Max angular speed (rad/s) for `cmd_vel`. |
| `cmd_timeout` | `0.5` | Seconds without a message (while moving) before a stop is published. `0` disables. |

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
The frontend sends JSON payloads to `ws://<HOST>:<PORT>/ws?token=<TOKEN>` at ~20Hz.
A handshake without the right token, or from a browser page of another origin, is refused with
HTTP 403. Messages larger than 1 KB close the connection.

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

It uses [Nipple.js](https://yoannmoi.net/nipplejs/) for the touch interface, served by the node
itself from `web_static/nipplejs.min.js` (v0.10.1, MIT): the page works without internet, and no
third-party script runs on the page that holds the token. To modify colors, size, or layout, edit the `index.html` file and rebuild the package:
```bash
colcon build --packages-select xpi_inputs
source install/setup.bash
```
*Note: The `index.html` is installed into the `share/xpi_inputs/web_static` directory during build.*

## 🛑 Safety
*   **Disconnect = stop.** When the WebSocket closes or drops, the node publishes a zero `cmd_vel` / `joy` immediately.
*   **Silent connection = stop.** If the connection stays open but no message arrives for `cmd_timeout` seconds while the robot is moving, the node publishes a stop.
*   **The page stops on leave.** Hiding the tab, switching apps or closing the page sends a zero command.
*   **Input is sanitized.** `lx` / `az` are clamped to `[-1, 1]`; `NaN`, `Infinity`, non-numbers and malformed JSON are treated as stop.
*   **Access control.**
    *   Loopback (`127.0.0.1`) by default; listening on the network is an explicit `host:=0.0.0.0`.
    *   Every WebSocket connection needs the token (compared in constant time).
    *   The `Origin` header is checked: a web page from another site open in your browser cannot
        connect to the robot (cross-site WebSocket hijacking), and neither can a DNS-rebinding page.
*   ⚠️ The connection is plain HTTP: anyone who can sniff the Wi-Fi can read the token. Use it on a
    trusted network (WPA2/3), or put it behind a TLS reverse proxy or a VPN.
