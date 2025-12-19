# UWB Beacons: Range-Only Navigation (SLAM)

This block implements **indoor positioning** using Ultra-Wideband (UWB) modules (e.g., **Decawave DWM1000**).
Unlike GPS, it provides centimeter-level accuracy by measuring the time-of-flight of radio pulses.

## 📡 The Concept: Autonomous Mapping
This block supports **Range-Only SLAM**:
1.  The robot starts with **zero knowledge** of beacon locations.
2.  As it moves, it records distances to discovered IDs.
3.  The system periodically optimizes the map, calculating both the **robot's path** and the **beacons' true coordinates**, even if initial estimates were wrong or missing.

## ⚡ Wiring (SPI)

| DWM1000 Pin | Raspberry Pi Pin | Description |
| :--- | :--- | :--- |
| **VCC** | 3.3V | Power. |
| **GND** | GND | Ground. |
| **SCK** | GPIO 11 | SPI Clock. |
| **MOSI** | GPIO 10 | SPI Data Out. |
| **MISO** | GPIO 9 | SPI Data In. |
| **CS** | GPIO 8 | Chip Select (CE0). |
| **IRQ** | GPIO 25 | Interrupt. |

## 🚀 Usage

### 1. Launch Ranging and SLAM
```bash
ros2 launch xpi_navigation beacon_slam.launch.py
```

### 2. Monitor Map
```bash
ros2 topic echo /navigation/beacon_map
```
*Output: List of Beacon IDs and their calculated [X, Y] coordinates.*

## 🧩 Topics Interface

### Publishers
*   `~/ranges` (`xpi_interfaces/msg/BeaconRangeArray`) - Raw distances to all visible beacons.
*   `~/pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`) - Estimated robot position.
*   `~/map` (`visualization_msgs/msg/MarkerArray`) - Positions of discovered beacons for RViz.
