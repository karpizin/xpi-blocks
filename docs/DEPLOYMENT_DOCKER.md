# XPI-Blocks: Deployment Guide (Docker)

This is the **recommended** way to run XPI-Blocks on your Raspberry Pi. Using Docker ensures all dependencies are correctly installed and provides a consistent environment.

## 🛠 Prerequisites

1.  **Raspberry Pi** with Ubuntu 22.04 or Raspberry Pi OS (64-bit).
2.  **Docker installed:**
    ```bash
    curl -sSL https://get.docker.com | sh
    sudo usermod -aG docker $USER
    # Logout and login again for changes to take effect
    ```

---

## 🚀 Running with Docker Compose (Easiest)

1.  **Download the `docker-compose.yml` file:**
    ```bash
    wget https://raw.githubusercontent.com/karpizin/xpi-blocks/main/docker-compose.yml
    ```
2.  **Start the container:**
    ```bash
    docker compose up -d
    ```
3.  **Enter the container:**
    ```bash
    docker exec -it xpi-blocks-container bash
    ```
    *Inside the container, ROS2 and the workspace are already sourced!*

---

## 📦 Pulling Directly from GHCR

You can pull the pre-built and pre-compiled image directly:
```bash
docker pull ghcr.io/karpizin/xpi-blocks:latest
```

### Run a specific launch file without entering the shell:
```bash
docker run -it --privileged --network host -v /dev:/dev ghcr.io/karpizin/xpi-blocks:latest ros2 launch xpi_sensors audio_level.launch.py
```

---

## 🛠 Local Build

If you made local changes and want to build your own image:
```bash
docker build -t xpi-blocks-local .
```

---

## 🔧 Hardware Access Notes

To ensure the Docker container can talk to your hardware, we use several critical flags:
*   `--privileged`: Gives the container access to GPIO, I2C, and SPI.
*   `-v /dev:/dev`: Mounts the host devices into the container.
*   `--network host`: Allows ROS2 nodes to communicate with other nodes on your Pi and with external tools like `rqt` or `rviz` on your laptop.

## ✅ Verification
Inside the container, run this to see if the hardware is accessible:
```bash
# Check I2C devices
i2cdetect -y 1

# List ROS2 topics
ros2 topic list
```
