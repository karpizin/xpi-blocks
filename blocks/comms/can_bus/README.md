# CAN Bus: MCP2515 Interface

This block enables **CAN Bus** communication using the **MCP2515** controller via SPI. In Linux, it is managed through **SocketCAN**, appearing as a network interface (`can0`).

## ⚙️ System Setup (Mandatory)

Before using this block, you must enable the SPI overlay in your Raspberry Pi configuration.

1.  Edit `/boot/firmware/config.txt` (or `/boot/config.txt`):
    ```bash
    sudo nano /boot/firmware/config.txt
    ```
2.  Add the following lines (adjust `oscillator` if your module uses 8MHz instead of 16MHz):
    ```text
    dtparam=spi=on
    dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
    ```
3.  Reboot the Pi.
4.  Bring up the interface:
    ```bash
    sudo ip link set can0 up type can bitrate 500000
    ```

## ⚡ Wiring (SPI)

| MCP2515 Pin | Raspberry Pi Pin | Description |
| :--- | :--- | :--- |
| **VCC** | 5V (Pin 2) | Power. |
| **GND** | GND (Pin 6) | Ground. |
| **SCK** | GPIO 11 (Pin 23) | SPI Clock. |
| **SI (MOSI)** | GPIO 10 (Pin 19) | SPI Data In. |
| **SO (MISO)** | GPIO 9 (Pin 21) | SPI Data Out. |
| **CS** | GPIO 8 (Pin 24) | Chip Select (CE0). |
| **INT** | GPIO 25 (Pin 22) | Interrupt (Matches overlay). |

## 🚀 Usage

### 1. Launch the Bridge
```bash
ros2 launch xpi_comms can_bus.launch.py
```

### 2. Send a CAN Frame
Send a message in JSON format: `{"id": 255, "data": [1, 2, 3, 4]}`
```bash
ros2 topic pub /can_bridge/tx std_msgs/msg/String "data: '{\"id\": 123, \"data\": [10, 20, 30]}'" -1
```

### 3. Receive CAN Frames
```bash
ros2 topic echo /can_bridge/rx
```

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `interface` | string | `can0` | SocketCAN interface name. |
| `bitrate` | int | `500000` | Bus speed (must match `ip link` setup). |

## 🧩 Topics Interface

### Subscribers
*   `~/tx` (`std_msgs/msg/String`) - JSON: `{"id": int, "data": [bytes], "is_extended": bool}`.

### Publishers
*   `~/rx` (`std_msgs/msg/String`) - JSON representing the received frame.
