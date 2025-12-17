# Telegram Bot Control

This block allows you to control and monitor your robot via a Telegram Bot. It is ideal for remote operation over the internet, as Telegram works without port forwarding or static IPs.

## 🎯 Features
*   **Menu Interface:** Custom buttons (Keyboards) for one-tap actions.
*   **Photo Request:** Send `📷 Photo` to get a snapshot from the robot's camera.
*   **Command Mapping:** Map buttons to ROS2 topics (`Bool`, `String`, etc.) via YAML config.
*   **Security:** Whitelist-based access control (User IDs).

## 🚀 Setup

### 1. Create a Bot
1.  Open **@BotFather** in Telegram.
2.  Send `/newbot`.
3.  Get your **HTTP API Token**.

### 2. Get your User ID
1.  Open **@userinfobot**.
2.  Get your numerical ID (e.g., `12345678`).

### 3. Configure
Edit `src/xpi_inputs/config/telegram_menu.yaml`:
```yaml
ros__parameters:
  token: "YOUR_TOKEN_HERE"
  allowed_users: [YOUR_ID]
```

### 4. Launch
```bash
ros2 launch xpi_inputs telegram.launch.py
```

## 📸 Camera Setup
To use the photo feature, you must have a camera node publishing to the bot's topic.
Example using standard `usb_cam` or our `camera` block:

```bash
ros2 run v4l2_camera v4l2_camera_node
ros2 run xpi_inputs telegram_bot_node --ros-args -r ~/image_raw:=/image_raw
```
*(Remap `~/image_raw` to your camera topic)*

## ⚙️ Logic
*   **Menu:** Defined in YAML as rows of strings.
*   **Actions:** Defined as `Trigger:topic:Type:Value`.
*   **Photo:** Triggering an action with `topic:internal` and type `photo` grabs the latest frame from `~/image_raw`.
