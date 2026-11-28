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
docker run -it --network host --device /dev/i2c-1 --device /dev/gpiomem \
    ghcr.io/karpizin/xpi-blocks:latest ros2 launch xpi_sensors bme280.launch.py
```

---

## 🛠 Local Build

If you made local changes and want to build your own image:
```bash
docker build -t xpi-blocks-local .
```

---

## 🔧 Hardware Access Notes

The container gets **only the devices it needs** (`devices:` in `docker-compose.yml`, `--device`
with `docker run`), not `--privileged` and the whole `/dev`: a privileged container is root on the
host, and the robot's control nodes listen on the network. Uncomment the devices your blocks use:

| Device | Used by |
|---|---|
| `/dev/i2c-1` | I2C sensors (BME280, MPU6050, AHT20, ...), PCA9685, SSD1306, I2C mux |
| `/dev/gpiomem` | GPIO through gpiozero: motor drivers, relays, buttons, PIR, sonar |
| `/dev/spidev0.0` | SPI: MAX7219 LED matrix, PMW3901, W25Qxx, LoRa |
| `/dev/ttyAMA0`, `/dev/ttyUSB0`, `/dev/ttyACM0` | UART: SBUS/CRSF, TF LiDARs, PMS5003, HC-12, Meshtastic, GPS |
| `/dev/snd` | audio blocks |
| `/dev/video0` | cameras |
| `/dev/mem` + `cap_add: [SYS_RAWIO]` | WS2812 (rpi_ws281x); gives full access to host memory, enable only if needed |

Docker refuses to start the container if a listed device does not exist on the host: enable the
interface (`sudo raspi-config` → Interface Options) or comment the line out. The PPM receiver talks
to the `pigpiod` daemon on the host over the network and needs no device.

`--network host` allows ROS2 nodes to communicate with other nodes on your Pi and with external
tools like `rqt` or `rviz` on your laptop.

## ⚠️ Missing hardware
A hardware node started without its device **stops with an error** (for example
`Cannot open /dev/i2c-1: ... or start the node with mock_hardware:=true`). It does not switch to
simulated data on its own. If you see this inside the container, the device is probably missing
from `devices:`. For a test run without hardware, pass `mock_hardware:=true` explicitly.

## ✅ Verification
Inside the container, run this to see if the hardware is accessible:
```bash
# Check I2C devices
i2cdetect -y 1

# List ROS2 topics
ros2 topic list
```
