# ArUco Markers: Visual Navigation

ArUco markers are square binary fiducial markers. This block allows a robot to detect markers using a camera and calculate its precise position and orientation (Pose) relative to the marker.

## 🏁 Preparation

### 1. Print Markers
*   **Generate:** Use [ArUco Generator](https://chev.me/arucogen/).
*   **Dictionary:** Use `4x4_50` (Standard for speed and reliability).
*   **Printing:** Use **Matte paper**. Glossy paper creates reflections that blind the sensor.
*   **Mounting:** Ensure the marker is perfectly flat. Use cardboard or a stiff backing.

### 2. Camera Calibration (CRITICAL)
Without calibration, the distance will be wrong and the rotation will "jitter".
*   Use the `camera_calibration` package in ROS2.
*   The `aruco_tracker` node listens to the `/camera_info` topic to get the calibration matrix automatically.
*   If no calibration is provided, the node falls back to generic values (unreliable!).

## 📐 Coordinate System
The poses are reported in the **Camera Optical Frame**:
*   **Z-axis (Blue):** Points forward (Distance to marker).
*   **X-axis (Red):** Points to the right.
*   **Y-axis (Green):** Points down.

If the marker is rotated, the **Orientation** (Quaternion) will reflect the rotation of the marker relative to the camera lens.

## 🚀 Usage

### 1. Launch Tracker
```bash
ros2 launch xpi_vision aruco_tracker.launch.py
```

### 2. Visualization (RViz)
Add a `PoseArray` display and set the topic to `/aruco_tracker/poses`. You will see 3D arrows appearing exactly where the markers are.

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `marker_size` | float | `0.1` | Physical side length of the marker in **meters**. |
| `dictionary_name` | string | `DICT_4X4_50` | ArUco dictionary (DICT_4X4_50, DICT_6X6_250, etc.). |

## 🧩 Topics Interface

### Subscribers
*   `~/image_raw` (`sensor_msgs/msg/Image`) - Raw camera feed.
*   `~/camera_info` (`sensor_msgs/msg/CameraInfo`) - Intrinsic camera parameters.

### Publishers
*   `~/poses` (`geometry_msgs/msg/PoseArray`) - 3D positions of all detected markers.
*   `~/debug_image` (`sensor_msgs/msg/Image`) - Feed with drawn markers, IDs, and 3D axes.

## 💡 Advanced Integration Scenarios

### Scenario A: Precision Docking
1.  Place a marker above the charging station.
2.  Robot approaches using Lidar until it sees the marker.
3.  Robot switches to "ArUco mode" and uses the `X` and `Z` coordinates to align itself perfectly with the pins.

### Scenario B: Global Relocalization
1.  Map the room and record the coordinates of fixed markers (e.g., Marker #1 is at [2.0, 5.0]).
2.  When the robot gets "lost" (odometry drift), it looks for Marker #1.
3.  Once seen, the robot can calculate its absolute position on the map by subtracting its relative pose from the marker's known position.

