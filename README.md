# XPI-Blocks: Open Source ROS2 Humble Library for Raspberry Pi Robotics & AI

`xpi-blocks` is a comprehensive, modular library of ROS2 components specifically optimized for **Raspberry Pi 4, 5, and Zero 2W**. Our mission is to lower the barrier to entry for robotics by providing perfectly documented "building blocks" that combine low-level hardware drivers with high-level intelligence (LLM/VLM).

## 🚀 Core Features
*   **80+ Hardware Drivers**: Ready-to-use nodes for popular sensors, actuators, and displays.
*   **ROS2 Hexapod Stack**: Full Inverse Kinematics and Gait Generation support.
*   **Embedded AI & LLM**: Native integration with OpenAI, Google Gemini, and Ollama for natural language robot control.
*   **Real-time Power Monitoring**: Support for UPS HATs and battery gauges (INA219, MAX17048, SW6106).
*   **Lidar & Computer Vision**: Drivers for LDROBOT, RPLidar, and OpenCV/Aruco integration.
*   **LoRa Mesh & Swarm**: Decentralized robot-to-robot communication via Meshtastic.

## 🏗 Block Architecture
Each "Block" in this library is atomic and includes:
*   **ROS2 Node**: Clean, idiomatic Python/C++ code.
*   **Hardware Interface**: Standardized HAL (GPIO, I2C, SPI, UART).
*   **Documentation**: Wiring diagrams, Bill of Materials (BOM), and Topic interfaces.
*   **Launch Files**: Configuration with sane defaults.

## 📂 Repository Structure
*   **`src/`**: ROS2 packages categorized by function (actuators, sensors, vision, etc.).
*   **`blocks/`**: Individual block documentation, diagrams, and specifications.
*   **`projects/`**: Integrated systems (e.g., Hexapod, Weather Station, Swarm Net).
*   **`docs/`**: Technical guides on AI, Lidar strategy, Testing, and Deployment.
*   **`scripts/`**: Setup utilities and udev rules.

## 🚀 Getting Started (Docker - Recommended)
The easiest way to run XPI-Blocks is via Docker, which guarantees a consistent environment.

```bash
# 1. Build or Pull the image
docker build -t xpi-blocks .

# 2. Run with hardware access
docker run -it --privileged --network host xpi-blocks
```

## 🛠 Tech Stack & Standards
*   **OS**: Ubuntu Server 22.04 LTS (64-bit) / Raspberry Pi OS.
*   **ROS2 Distro**: Humble Hawksbill (LTS).
*   **GPIO Access**: `gpiozero` (Standardized HAL).
*   **I2C Access**: `smbus2` via `xpi_commons`.
*   **Universal Input**: `joy_mapper_node` for Gamepads, Keyboards, and Mice.

## 📋 Prerequisites
Before connecting hardware, ensure:
1.  **I2C/SPI Enabled**: via `sudo raspi-config`.
2.  **Permissions**: User must be in `gpio`, `i2c`, and `dialout` groups.
3.  **Power**: Use a stable 5V/3A+ power supply for the Raspberry Pi.

## 🧪 Testing
```bash
# Run all tests using Mock hardware
colcon test --packages-select xpi_sensors
colcon test-result --verbose
```

## 📈 Project Roadmap & Capabilities
*   [🚀 Project Capabilities](docs/CAPABILITIES.md) — High-level feature overview.
*   [🎮 Desktop Simulation Guide](docs/SIMULATION_DESKTOP.md) — Run XPI-Blocks in Gazebo on your PC.
*   [📚 Library Index](docs/BLOCKS_INDEX.md) — Direct links to all implemented blocks.
*   [🗺️ Device Roadmap](ROADMAP.md) — List of targeted and completed devices.
*   [🤖 LLM Scenarios](docs/LLM_VLM_SCENARIOS.md) — Plan for AI integration.

## 🤝 Contact
Developer: Viacheslav Karpizin (viacheslav.karpizin@gmail.com)
