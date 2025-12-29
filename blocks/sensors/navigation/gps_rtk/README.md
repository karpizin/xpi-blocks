# RTK GPS Block (High Precision Positioning)

This block provides centimeter-level positioning accuracy using Real-Time Kinematic (RTK) technology.

## 🛰️ What is RTK and how does it differ from standard GPS?
Standard GPS receivers have an error margin of **2 to 5 meters** due to atmospheric distortions.
**RTK (Real-Time Kinematic)** uses an additional data stream (corrections) from a stationary **Base Station** with known coordinates. This eliminates errors and achieves an accuracy of **1-2 centimeters**.

### Fix Types:
1.  **NO FIX (0)**: No satellites found.
2.  **GPS FIX (1)**: Standard precision (2-5 meters).
3.  **FLOAT (2)**: Corrections received, but phase ambiguity is not yet resolved (~20-50 cm accuracy).
4.  **FIXED (3)**: Full phase resolution achieved. Centimeter precision (**1-3 cm**).

## 🔌 Connection
RTK modules (e.g., u-blox ZED-F9P) are typically connected via UART or USB.

## 🚀 Usage

### 1. Launch RTK GPS only
If you have a local base station or radio link already providing corrections to the serial port:
```bash
ros2 launch xpi_sensors gps_rtk.launch.py port:=/dev/ttyUSB0
```

### 2. Launch with NTRIP (Centimeter accuracy via Internet)
To receive corrections from a network provider (e.g., RTK2GO):
```bash
ros2 launch xpi_sensors gps_rtk.launch.py ntrip_mount:=YOUR_MOUNTPOINT
```

## ⚙️ Parameters

### gps_rtk_node
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `port` | string | `/dev/ttyUSB0` | Serial port path |
| `baudrate` | int | `38400` | Baud rate (ZED-F9P default is 38400) |

### ntrip_client_node
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `host` | string | `rtk2go.com` | NTRIP Caster address |
| `port` | int | `2101` | Caster port |
| `mountpoint`| string | `""` | Mountpoint name (Required) |

## 📡 ROS2 Interface

### Publishers
*   `~/fix` (`sensor_msgs/NavSatFix`): High-precision position data.
*   `~/rtk_status` (`std_msgs/Int32`): 0=No Fix, 1=3D, 2=Float RTK, 3=Fixed RTK.

### Subscribers
*   `/rtk/corrections` (`std_msgs/String`): RTCM correction stream (received as hex).

## 🛠 Troubleshooting
*   **Stuck in FLOAT (2):** Ensure you have a clear view of the sky and a stable internet connection for NTRIP. It can take 1-5 minutes to reach FIXED status.
*   **Permission Denied:** Ensure your user is in the `dialout` group: `sudo usermod -aG dialout $USER`.
*   **No Data:** Check if `pyubx2` is installed: `pip install pyubx2`.