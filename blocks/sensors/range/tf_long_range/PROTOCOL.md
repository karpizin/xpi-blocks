# Benewake TF Series: Binary Protocol & Data Processing

This document describes the low-level communication protocol used by TF-Luna, TFmini (Plus), TF02-Pro, and TF03-100 LiDARs.

## 📦 Data Packet Format (Standard 9-Byte)

All TF sensors send data in continuous packets of 9 bytes via UART.

| Byte | Value | Name | Description |
| :--- | :--- | :--- | :--- |
| 0 | `0x59` | Header 1 | Fixed frame header |
| 1 | `0x59` | Header 2 | Fixed frame header |
| 2 | `Dist_L` | Distance Low | Lower 8 bits of distance |
| 3 | `Dist_H` | Distance High | Upper 8 bits of distance |
| 4 | `Str_L` | Strength Low | Lower 8 bits of signal strength |
| 5 | `Str_H` | Strength High | Upper 8 bits of signal strength |
| 6 | `Temp_L` | Temp Low | Lower 8 bits of sensor temperature |
| 7 | `Temp_H` | Temp High | Upper 8 bits of sensor temperature |
| 8 | `Checksum` | Checksum | Sum of bytes 0-7 (lowest 8 bits) |

### Calculations:
*   **Distance (cm):** `(Byte 3 << 8) | Byte 2`
*   **Strength:** `(Byte 5 << 8) | Byte 4` (Values < 100 are unreliable)
*   **Temperature (°C):** `((Byte 7 << 8) | Byte 6) / 8 - 256`

---

## 🌩️ Processing LiDAR Data in ROS2

While these sensors are 1D (Point rangefinders), their data is often used as a building block for more complex systems.

### 1. Generating a Point Cloud
To convert multiple 1D distances into a 3D Point Cloud, you typically combine the sensor data with:
*   **Servo/Gimbal:** Rotating the sensor to scan an area.
*   **Transformations:** Using `tf2` to project the point based on the sensor's orientation.

### 2. Useful ROS2 Packages
*   **[pointcloud_to_laserscan](https://index.ros.org/p/pointcloud_to_laserscan/):** Convert 3D data into 2D scans for SLAM.
*   **[filters](https://index.ros.org/p/filters/):** For smoothing noisy range data (Median, Mean filters).
*   **[laser_filters](https://index.ros.org/p/laser_filters/):** Advanced geometric filtering.

### 3. Visualization in Rviz2
To see the range data:
1.  Run the node.
2.  Open `rviz2`.
3.  Add a **Range** display.
4.  Set the topic to `/tf_lidar/range`.
5.  Set the **Fixed Frame** to `tf_lidar_link`.

---

## 📚 External Resources
*   **[Benewake Official Downloads](https://en.benewake.com/support):** Manuals and GUI tools for PC.
*   **[Point Cloud Library (PCL)](https://pointclouds.org/):** The industry standard for processing massive point cloud data.
*   **[Open3D](http://www.open3d.org/):** Modern library for 3D data processing (Python/C++).
