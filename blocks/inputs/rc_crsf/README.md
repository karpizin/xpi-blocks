# CRSF Receiver Input (Crossfire / ELRS)

This block provides a ROS2 driver for decoding CRSF protocol data from an RC receiver (like TBS Crossfire or ExpressLRS) connected via UART. It publishes decoded channel values as `sensor_msgs/Joy` messages.

## 📦 Bill of Materials
*   Raspberry Pi
*   CRSF-compatible RC Receiver (e.g., TBS Crossfire, ExpressLRS)
*   Jumper Wires

## 🔌 Wiring
Connect your CRSF receiver to the Raspberry Pi's UART pins.

| RC Receiver CRSF | Raspberry Pi | Note |
|------------------|--------------|-----------------------------------|
| TX               | GPIO 14 (TXD) | Connects to RPi's TXD (UART)     |
| RX               | GPIO 15 (RXD) | Connects to RPi's RXD (UART)     |
| GND              | GND          | Common Ground                     |
| VCC              | 3.3V/5V      | Power for Receiver (check receiver spec!) |

**Important UART Configuration on Raspberry Pi:**
The primary UART (`/dev/ttyS0` or `/dev/serial0`) on Raspberry Pi needs to be configured:

1.  **Disable Console UART:**
    ```bash
    sudo raspi-config
    # Interface Options -> P6 Serial Port -> Would you like a login shell to be accessible over serial? -> No
    # Interface Options -> P6 Serial Port -> Would you like the serial port hardware to be enabled? -> Yes
    ```
    This ensures `ttyS0` is available.
2.  **Ensure `serial0` points to `ttyS0`:** Check `/boot/firmware/config.txt` and ensure `dtoverlay=uart0` (or `enable_uart=1`) is set, and `dtoverlay=disable-bt` might be needed if Bluetooth uses `uart0`.

## 🚀 Quick Start
1.  **Perform UART Configuration** as described above.
2.  **Install `pyserial`**:
    ```bash
    sudo apt update
    sudo apt install python3-pyserial
    ```
3.  **Launch the CRSF driver**:
    ```bash
    ros2 launch xpi_inputs crsf.launch.py uart_port:=/dev/ttyS0
    ```
    (Adjust `uart_port` if using a different port, like `/dev/ttyAMA0` or a USB-to-UART adapter).

## 📡 Interface
### Publishers
*   `~/joy` (`sensor_msgs/Joy`): Publishes decoded CRSF channel values.
    *   `axes`: Array of 16 floats, normalized to `[-1.0, 1.0]`.
    *   `buttons`: Not typically used for CRSF.

### Parameters
*   `uart_port` (string, default: `/dev/ttyS0`): UART device file.
*   `baud_rate` (int, default: `420000`): Standard CRSF baud rate.
*   `publish_rate` (float, default: `100.0`): Frequency to publish `Joy` messages.
*   `mock_hardware` (bool, default: `false`): Run in mock mode for testing.

## ✅ Verification
1.  Launch the driver with your CRSF receiver connected and powered on.
2.  In a new terminal, monitor the `/crsf_receiver/joy` topic:
    ```bash
    ros2 topic echo /crsf_receiver/joy
    ```
3.  Move sticks and switches on your RC transmitter; you should see the `axes` values change.

## ⚠️ Troubleshooting
*   **No data / Garbage data?**
    *   Double-check UART wiring (TX of receiver to RX of RPi, RX of receiver to TX of RPi).
    *   Verify UART configuration on RPi (`raspi-config`).
    *   Ensure correct baud rate.
*   **Permissions error?**
    *   Add your user to the `dialout` group: `sudo usermod -a -G dialout $USER`. Log out and back in.
    *   Ensure the UART port (`/dev/ttyS0`) has read/write permissions.
