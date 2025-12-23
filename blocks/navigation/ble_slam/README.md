# BLE SLAM & Localization

This block implements **Indoor Positioning** using Bluetooth Low Energy (BLE) beacons. It supports two modes of operation:

1.  **LOCALIZATION (Trilateration):** The robot knows the coordinates of the beacons (from a YAML map) and calculates its own position $(x, y)$.
2.  **MAPPING (Experimental):** The robot does not know where the beacons are and attempts to map them while moving.

## 🧠 How it Works

### Localization (Trilateration)
The node receives distances to 3 or more known beacons. It solves a **Non-linear Least Squares** problem to find the point $(x, y)$ that minimizes the error between measured distances and calculated distances to the anchors.
*   **Math:** `minimize sum( || Robot - Beacon_i || - MeasuredDist_i )^2`
*   **Solver:** `scipy.optimize.least_squares`

### Mapping
This is a complex problem (SLAM). The current implementation provides placeholders for GraphSLAM. It collects odometry and beacon ranges but does not yet perform full graph optimization (which requires a heavy backend like g2o or GTSAM).

## 📦 Setup

### 1. Place Beacons
Place at least 3 (ideally 4+) BLE beacons in your room. Ensure they are not in a straight line (collinear). Measure their coordinates relative to a corner (0,0).

### 2. Create Map File
Edit `xpi_navigation/config/known_beacons.yaml`:
```yaml
beacons:
  "AA:BB:CC:DD:EE:01": [0.0, 0.0, 1.5]
  "AA:BB:CC:DD:EE:02": [5.0, 0.0, 1.5]
  "AA:BB:CC:DD:EE:03": [2.5, 4.0, 1.5]
```

### 3. Install Dependencies
```bash
pip3 install scipy
```

## 🚀 Usage

**Start Localization:**
```bash
ros2 launch xpi_navigation ble_slam.launch.py mode:=LOCALIZATION beacons_file:=/path/to/your/map.yaml
```

**Start Ranging Only (Debugging):**
```bash
ros2 run xpi_navigation ble_ranging_node
```

## 📡 Interface

### Publishers
*   `~/pose` (`geometry_msgs/PoseWithCovarianceStamped`): The calculated position of the robot in the map frame.

### Parameters
*   `mode`: `LOCALIZATION` or `MAPPING`.
*   `beacons_file`: Absolute path to the YAML map file.

## ⚠️ Limitations
*   **Noise:** BLE signal (RSSI) fluctuates heavily. The position output will "jitter". Use a robot-side EKF (e.g., `robot_localization` package) to fuse this `~/pose` with Wheel Odometry for a smooth path.
*   **2D Only:** Currently solves for X and Y. Z is assumed to be 0 or fixed.
