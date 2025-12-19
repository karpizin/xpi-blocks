# ArUco Markers: Visual Navigation

ArUco markers are square binary fiducial markers. This block allows a robot to detect markers using a camera and calculate its precise position and orientation (Pose) relative to the marker.

**Use Cases:**
*   **Docking:** Finding a charging station.
*   **Precision Landing:** For drones.
*   **Room Labeling:** Knowing exactly which room the robot is in.

## 🏁 Preparation

### 1. Print Markers
Generate markers at [chilipeppr.com/aruco](https://chilipeppr.com/aruco) or similar.
*   **Dictionary:** 4x4_50 (Default).
*   **Size:** Know the physical size (e.g., 10cm) – you need to enter this in the parameters.

### 2. Calibrate Camera (Critical!)
To get accurate distance/rotation, your camera must be calibrated.
Save your `camera_matrix` and `dist_coeffs` to a YAML file.

## 🚀 Usage

### 1. Launch Tracker
```bash
ros2 launch xpi_vision aruco_tracker.launch.py
```

### 2. Verify Data
```bash
ros2 topic echo /vision/aruco_poses
```
*Output: List of marker IDs and their relative 3D positions.*

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `marker_size` | float | `0.1` | Physical size of the marker in meters. |
| `dictionary` | string | `DICT_4X4_50` | ArUco dictionary to use. |
| `camera_info_url` | string | `""` | Path to calibration YAML file. |

## 🧩 Topics Interface

### Subscribers
*   `~/image_raw` (`sensor_msgs/msg/Image`) - Raw camera feed.

### Publishers
*   `~/poses` (`geometry_msgs/msg/PoseArray`) - 3D positions of all detected markers.
*   `~/debug_image` (`sensor_msgs/msg/Image`) - Feed with drawn markers and axes.
