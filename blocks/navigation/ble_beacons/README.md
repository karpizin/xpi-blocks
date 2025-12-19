# BLE Beacons: Low-Cost Indoor Ranging

This block uses **Bluetooth Low Energy (BLE)** signal strength (**RSSI**) to estimate distances to beacons.
While much cheaper than UWB, it is significantly noisier due to signal reflections and interference.

## 📡 The Concept: RSSI to Distance
Distance is estimated using the **Log-Distance Path Loss Model**:
`d = 10 ^ ((Measured_Power - RSSI) / (10 * N))`
*   **Measured_Power:** RSSI at 1 meter (Calibration constant).
*   **N:** Path loss exponent (usually 2.0 for free space, up to 4.0 for indoors).

## 🚀 Usage

### 1. Launch BLE SLAM
```bash
ros2 launch xpi_navigation ble_slam.launch.py
```

## 🚧 Future Improvements (The "Fix-it" List)
Because BLE RSSI is extremely unstable, the following techniques should be implemented to make this usable:

1.  **Kalman Filtering:** Apply a 1D Kalman filter to every beacon's distance stream to smooth out rapid jumps.
2.  **Radio Fingerprinting:** Instead of geometry, record "signal signatures" at specific locations and use AI to match current signals to the map.
3.  **IMU Fusion:** Rely heavily on Wheel Odometry and Gyroscope, using BLE only as a very "soft" hint to prevent long-term drift.
4.  **Multipate Rejection:** Ignore sudden drops in signal strength (which usually mean someone walked between the robot and the beacon).

## 🧩 Topics Interface

### Publishers
*   `~/raw_ranges` (`std_msgs/msg/String`) - JSON: `{"MAC_ADDR": distance_meters}`.
