# SBUS Receiver Input

This block provides a ROS2 driver for decoding SBUS protocol data from an RC receiver connected via UART. It publishes decoded channel values as `sensor_msgs/Joy` messages.

## 📦 Bill of Materials
*   Raspberry Pi
*   SBUS-compatible RC Receiver (e.g., FrSky, Futaba, Crossfire/ELRS with SBUS output)
*   Jumper Wires
*   **Optional:** SBUS Inverter (if your receiver outputs inverted SBUS and you're connecting directly to RPi UART RX, which is non-inverting). Many modern receivers output non-inverted SBUS.

## 🔌 Wiring
Connect your SBUS receiver to the Raspberry Pi's UART pins.

| RC Receiver SBUS | Raspberry Pi | Note |
|------------------|--------------|------------------------------|
| SBUS Out         | GPIO 15 (TXD) | Connected to RPi's RXD (UART) |
| GND              | GND          | Common Ground                |
| VCC              | 3.3V/5V      | Power for Receiver (check receiver spec!) |

**Important UART Configuration on Raspberry Pi:**
By default, the primary UART (`/dev/ttyS0` or `/dev/serial0`) on Raspberry Pi is often used for console output or Bluetooth. You *must* disable this to use it for SBUS.

1.  **Disable Console UART:**
    ```bash
    sudo raspi-config
    # Interface Options -> P6 Serial Port -> Would you like a login shell to be accessible over serial? -> No
    # Interface Options -> P6 Serial Port -> Would you like the serial port hardware to be enabled? -> Yes
    ```
    This ensures `ttyS0` is available.
2.  **Ensure `serial0` points to `ttyS0`:** On some Pis, `serial0` might be linked to Bluetooth. Check `/boot/firmware/config.txt` and ensure `dtoverlay=uart0` (or `enable_uart=1`) is set, and `dtoverlay=disable-bt` might be needed if Bluetooth uses `uart0`.

## 🚀 Quick Start
1.  **Perform UART Configuration** as described above.
2.  **Install `pyserial`**:
    ```bash
    sudo apt update
    sudo apt install python3-pyserial
    ```
3.  **Launch the SBUS driver**:
    ```bash
    ros2 launch xpi_inputs sbus.launch.py uart_port:=/dev/ttyS0
    ```
    (Adjust `uart_port` if using a different port, like `/dev/ttyAMA0` or a USB-to-UART adapter).

## 📡 Interface
### Publishers
*   `~/joy` (`sensor_msgs/Joy`): Publishes decoded SBUS channel values.
    *   `axes`: Array of 16 floats, normalized to `[-1.0, 1.0]`.
    *   `buttons`: `[0]` = Failsafe status (1 if failsafe active), `[1]` = Frame Lost status (1 if frame lost).

### Parameters
*   `uart_port` (string, default: `/dev/ttyS0`): UART device file.
*   `baud_rate` (int, default: `100000`): SBUS standard baud rate.
*   `publish_rate` (float, default: `50.0`): Frequency to publish `Joy` messages.
*   `mock_hardware` (bool, default: `false`): Run in mock mode for testing.
*   `invert_sbus` (bool, default: `false`): Set to `true` if your receiver outputs inverted SBUS (rarely needed for modern non-inverted receivers).

## ✅ Verification
1.  Launch the driver with your SBUS receiver connected and powered on.
2.  In a new terminal, monitor the `/sbus_receiver/joy` topic:
    ```bash
    ros2 topic echo /sbus_receiver/joy
    ```
3.  Move sticks and switches on your RC transmitter; you should see the `axes` values change.

## ⚠️ Troubleshooting
*   **No data / Garbage data?**
    *   Double-check UART wiring (TXD from RPi to RX on SBUS receiver, or vice-versa depending on SBUS output type).
    *   Verify UART configuration on RPi (`raspi-config`).
    *   Ensure correct baud rate, parity, stop bits in the launch file.
    *   Check for SBUS inversion: try setting `invert_sbus:=true` if you suspect inverted SBUS data.
*   **Permissions error?**
    *   Add your user to the `dialout` group: `sudo usermod -a -G dialout $USER`. Log out and back in.
    *   Ensure the UART port (`/dev/ttyS0`) has read/write permissions.
