# Bluetooth Low Energy (BLE) Bridge

This block turns your Raspberry Pi into a **BLE Peripheral**, allowing smartphones (iOS/Android) and other devices to connect directly to the robot without Wi-Fi.

It supports:
1.  **Direct Control:** Sending velocity commands (`/cmd_vel`) from a phone app.
2.  **Telemetry:** Streaming robot status (battery, etc.) back to the phone.

## 🧠 GATT Service Architecture

The robot exposes a custom GATT Service with the following UUIDs:

*   **Service UUID:** `A07498CA-AD5B-474E-940D-16F1FBE7E8CD`
    *   **Command Characteristic (Write):** `51FF12BB-3ED8-46E5-B4F9-D64E2FEC021B`
        *   Write JSON strings here to control the robot.
    *   **Telemetry Characteristic (Notify/Read):** `51FF12BB-3ED8-46E5-B4F9-D64E2FEC021C`
        *   Subscribe to this to receive JSON status updates.

## 📦 Prerequisites
*   Raspberry Pi 3B+, 4, 5, or Zero 2W (Built-in Bluetooth).
*   **System Dependencies:**
    ```bash
    sudo apt-get install bluez
    ```
*   **Python Dependencies:**
    ```bash
    pip3 install bless
    ```

## 🚀 Usage

**Launch the bridge:**
```bash
ros2 launch xpi_comms ble_bridge.launch.py
```

**Permission Note:**
Running a BLE Server typically requires root privileges to access the HCI controller. If it fails, try:
```bash
sudo setcap 'cap_net_raw,cap_net_admin+eip' $(eval readlink -f `which python3`)
```
Or run simply with `sudo` (not recommended for ROS2 generally, but often necessary for BLE).

## 📱 How to Connect (Smartphone)

1.  Download a BLE Debugger app:
    *   **nRF Connect** (Android/iOS) - Highly recommended.
    *   **LightBlue** (iOS).
2.  Scan for devices. Look for **"XPI-Robot"**.
3.  Connect.
4.  Open the Service `A074...`.

### Controlling the Robot
Select the **Command Characteristic** (`...021B`) and write a JSON string (UTF-8):
*   `{"lx": 0.5, "az": 0.0}` -> Move Forward at 0.5 m/s.
*   `{"lx": 0.0, "az": 1.0}` -> Rotate Left.
*   `{"led": "ON"}` -> Custom command (published to `~/incoming_command`).

### Reading Telemetry
Select the **Telemetry Characteristic** (`...021C`) and enable **Notifications** (subscribe).
You will see a stream of JSON data:
*   `{"bat": 12.4, "status": "ok"}`

## ⚠️ Troubleshooting
*   **"Permission Denied"**: See the Permission Note above. Linux BlueZ stack is strict about who can control the radio.
*   **"Device not found"**: Ensure the ROS2 node is running and `bluetooth.service` is active on the Pi (`systemctl status bluetooth`).
