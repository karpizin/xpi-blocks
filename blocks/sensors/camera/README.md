# Camera (V4L2 / USB / CSI)

This block enables video streaming from USB webcams and Raspberry Pi Cameras (CSI) using the standard Video4Linux2 (V4L2) interface. It is the foundational block for Computer Vision (CV) and Vision-Language Model (VLM) tasks.

## 📦 Prerequisites

### 1. Install ROS2 Driver
We use the standard `v4l2_camera` package, which is reliable and efficient.
```bash
sudo apt update
sudo apt install ros-humble-v4l2-camera
sudo apt install ros-humble-image-transport-plugins # For compressed image transport
```

### 2. User Permissions
The user running the node must be in the `video` group to access `/dev/video*` devices.
```bash
sudo usermod -aG video $USER
```
*Note: You may need to logout and login again for this to take effect.*

## 🔌 Hardware Setup

### USB Webcams
1.  Plug in your USB camera.
2.  Check if it's detected:
    ```bash
    ls -l /dev/video*
    ```
    You should see `/dev/video0` (and often `/dev/video1` which is metadata).

### Raspberry Pi Camera Module (CSI)
On modern Raspberry Pi OS (Bullseye/Bookworm), the camera stack moved to **libcamera**. The standard `/dev/video0` might not appear automatically for CSI cameras unless legacy support is enabled, OR you use a `libcamera` based ROS2 driver.

**Option A: Use `v4l2_camera` (Legacy Mode / USB approach)**
This is the simplest way if you want a uniform interface for both USB and Pi Cameras.
1.  Edit `/boot/config.txt` (or `/boot/firmware/config.txt` on Bookworm):
    ```ini
    dtoverlay=imx219  # Or your specific sensor model
    ```
2.  If you have issues, try enabling "Legacy Camera Support" in `raspi-config` (Interface Options -> Legacy Camera), though this is being deprecated.

**Option B: Use `camera_ros` (Native libcamera)**
If you are strictly using a Pi Camera on a new OS and want the best performance, consider using the [camera_ros](https://github.com/christianrauch/camera_ros) package instead of this block.

*This block focuses on the generic `v4l2` approach which works for 99% of USB cameras and many configured Pi cameras.*

## 🚀 Usage

### 1. Launch the Camera
```bash
ros2 launch camera.launch.py
```
*(Copy `camera.launch.py` from this folder to your workspace or run it directly)*

### 2. Verify Output
List the topics. You should see raw images and compressed streams.
```bash
ros2 topic list
```
Expected output:
*   `/camera/image_raw`
*   `/camera/image_raw/compressed`
*   `/camera/camera_info`

### 3. View the Stream
Use `rqt_image_view` (if you have a desktop environment) or stream it to a web browser using a web bridge.
```bash
ros2 run rqt_image_view rqt_image_view
```

## ⚙️ Parameters (in launch file)

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `video_device` | `/dev/video0` | Path to the camera device file. |
| `image_size` | `[640, 480]` | Resolution (Width, Height). Lower res = higher FPS & lower latency. |
| `time_per_frame` | `[1, 30]` | Frame rate fraction (numerator, denominator). `[1, 30]` = 30 FPS. |
| `pixel_format` | `YUYV` | `YUYV` (uncompressed) or `MJPG` (compressed on camera). MJPG saves USB bandwidth but adds CPU load for decoding. |
| `camera_frame_id` | `camera_link` | TF Frame ID for the optical center. |

## 🛠 Troubleshooting

*   **`Permission denied: '/dev/video0'`**:
    *   Did you add your user to the `video` group?
    *   Try `ls -l /dev/video0` to see permissions.
*   **`device not found`**:
    *   Is the camera plugged in?
    *   Run `v4l2-ctl --list-devices` (install `v4l-utils`) to see available cameras.
*   **Laggy Video / High Latency**:
    *   Reduce `image_size`.
    *   Try `pixel_format: 'MJPG'` if your camera supports it.
    *   Ensure you are using `image_transport` (compressed topic) if viewing over Wi-Fi.
