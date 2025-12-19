# XPI-Blocks: The ROS2 & Raspberry Pi Building Blocks Library

## 🎯 Mission
To create a **comprehensive, modular, and perfectly documented** library of examples (blocks) for ROS2 and Raspberry Pi. This project aims to **lower the barrier to entry for robotics and enable intelligent behaviors through LLM/VLM integration**.
**Goal:** A developer should be able to pick a "block" (code + diagram), connect the peripheral, and have a working ROS2 node in 5 minutes.

## 🏗 Block Architecture
Each example in this library is an atomic "Block" containing:
*   **ROS2 Node:** Clean, idiomatic code (Python/C++).
*   **Hardware Interface:** Hardware abstraction (GPIO, I2C, SPI, UART, 1-Wire) or intelligent LLM/VLM interfaces.
*   **Launch file:** Launch configuration with sane defaults.
*   **Documentation:**
    *   Wiring Diagram.
    *   Bill of Materials (BOM).
    *   Topic Interfaces (input/output).
    *   Verification Command (one-liner).

## 📂 Repository Structure
```text
xpi-blocks/
├── docs/               # Global documentation and guides (e.g., ARCHITECTURE.md, ROADMAP.md, LLM_VLM_SCENARIOS.md, NEXT_KEY_TASKS.md)
├── blocks/             # Individual block documentation (e.g., wiring, usage)
│   ├── inputs/         # Input devices (joystick, keyboard, RC receivers)
│   ├── actuators/      # Actuators (motors, servos, relays, steppers, LEDs, displays)
│   ├── sensors/        # Sensors (IMU, lidar, range, environment, analog, 1-wire, digital-input)
│   ├── comms/          # Communication bridges (Serial, UDP, etc.)
│   └── llm/            # LLM/VLM integration blocks
├── src/                # ROS2 packages
│   ├── xpi_commons/    # Common utilities, HAL for GPIO/I2C
│   ├── xpi_inputs/     # Input device drivers
│   ├── xpi_actuators/  # Actuator drivers
│   ├── xpi_sensors/    # Sensor drivers
│   ├── xpi_comms/      # Communication nodes
│   └── xpi_llm/        # LLM/VLM integration nodes
└── scripts/            # Setup utilities (Docker, udev rules)
```

## 🚀 Getting Started

1.  **Clone the repository:**
    ```bash
    git clone git@github.com:karpizin/xpi-blocks.git
    cd xpi-blocks
    ```
2.  **Install ROS2 dependencies:**
    ```bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```
3.  **Build the workspace:**
    ```bash
    colcon build --symlink-install
    ```
4.  **Source the workspace:**
    ```bash
    . install/setup.bash
    ```
    Now you can run the example nodes and launch files!

## 🛠 Tech Stack & Standards
*   **Primary OS:** Ubuntu Server 22.04 LTS (64-bit) for Raspberry Pi.
*   **ROS2 Distro:** Humble Hawksbill (LTS).
*   **Compatibility:** Code is forward-compatible with Jazzy/Ubuntu 24.04 where possible.
*   **Language:** Python 3.10+ (primary), C++ (time-critical).
*   **GPIO Access:** `gpiozero` (Standardized HAL).
*   **I2C Access:** `smbus2` via `xpi_commons` (Standardized HAL) and libraries like `luma.oled`.
*   **SPI Access:** `luma.led_matrix` for displays.
*   **UART Access:** `pyserial` for RC receivers.
*   **1-Wire Access:** Kernel modules (`w1_gpio`, `w1_therm`).
*   **LLM/VLM:** Flexible backend support (Gemini, OpenRouter, Ollama) via `xpi_llm`.
*   **Universal Input:** `joy_mapper_node` converts any Joystick, Mouse, or Keyboard into standard robot commands (`Twist`, `Bool`, `Float32`) via YAML config.
*   **Containerization:** `Dockerfile` support for rapid deployment.

## 📋 Prerequisites
Before using these blocks, ensure you have:
1.  **Raspberry Pi 4 or 5** (Zero 2W is also supported).
2.  **Ubuntu 22.04 Installed** (Official RPi Imager image).
3.  **ROS2 Humble Desktop/Base** installed and sourced.
    ```bash
    source /opt/ros/humble/setup.bash
    ```
4.  **GPIO Privileges:** User must be in `gpio` group (or `dialout` for UART, `i2c` for I2C, `spi` for SPI, `users` for 1-Wire).
5.  **I2C Enabled:** `sudo raspi-config` -> Interface Options -> I2C.
6.  **SPI Enabled:** `sudo raspi-config` -> Interface Options -> SPI.
7.  **1-Wire Enabled:** `sudo raspi-config` -> Interface Options -> 1-Wire.
8.  **UART Configured:** For RC receivers, UART needs to be freed from console (`sudo raspi-config`).

## 📝 Documentation Guidelines
*   **README Driven:** Documentation is written before code.
*   **Visuals:** Photos of real wiring + Fritzing/Schematic diagrams.
*   **Troubleshooting:** Dedicated section for common issues in every block.

## 📈 Project Status & Roadmap
*   **[🚀 PROJECT CAPABILITIES (Full List of Features)](docs/CAPABILITIES.md)** - Start here for a high-level overview.
*   **[📚 LIBRARY INDEX (Detailed Links)](docs/BLOCKS_INDEX.md)** - Index of all implemented blocks with direct links to docs.
*   See [ROADMAP.md](ROADMAP.md) for a list of all targeted devices.
*   See [LLM_VLM_SCENARIOS.md](docs/LLM_VLM_SCENARIOS.md) for the LLM/VLM integration plan.
*   See [NEXT_KEY_TASKS.md](NEXT_KEY_TASKS.md) for high-priority future tasks.
