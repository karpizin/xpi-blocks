# XPI-Blocks: LiDAR & Depth Sensing Strategy

This document outlines the approach for integrating light detection and ranging (LiDAR) sensors into the XPI-Blocks ecosystem.

## 📏 Classification of Sensors

### 1. 1D Rangefinders (Point Sensors)
Best for: Altitude hold, door detection, obstacle avoidance in a specific direction.
*   **Budget:** VL53L1X (ToF), HC-SR04 (Sonar).
*   **Outdoor/Long Range:** TFmini Plus, TF-Luna, Garmin Lidar-Lite.

### 2. 2D Scanning LiDARs (360° Mapping)
Best for: SLAM (Simultaneous Localization and Mapping), navigation, floor cleaning robots.
*   **Mechanical:** RPLIDAR A1/A2, LDROBOT LD19.
*   **Solid-State (MEMS/Flash):** CE30-A.

### 3. 3D Depth Cameras & LiDARs (Spatial Awareness)
Best for: Object recognition, complex obstacle avoidance, gesture control.
*   **Stereo Depth:** Luxonis OAK-D (recommended), Intel RealSense.
*   **3D LiDAR:** Livox Mid-360, HPS-3D160.

---

## 🛠 Integration Priority

### Phase 1: High-Frequency 1D (Current Focus)
Ensure robust UART/I2C drivers for TFmini and TF-Luna to support high-speed movement.

### Phase 2: Spatial AI (OAK-D)
Integrating Luxonis OAK-D is a top priority for the **MCP Agent**. It provides not just depth, but semantic information (e.g., "Person at 2.5m, 30° Left").

### Phase 3: High-End SLAM
Providing standard bridges for Livox and RPLIDAR using official ROS2 drivers but with simplified XPI-Blocks configuration.

---

## 🔌 Hardware Interface Standards
*   **UART:** Preferred for 1D rangefinders due to wiring simplicity.
*   **SPI/I2C:** Used for short-range ToF.
*   **USB 3.0 / Ethernet:** Mandatory for 3D/High-density data.
