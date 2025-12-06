# XPI-Blocks: The ROS2 & Raspberry Pi Building Blocks Library

## 🎯 Mission
To create a **comprehensive, modular, and perfectly documented** library of examples (blocks) for ROS2 and Raspberry Pi.
**Goal:** Lower the barrier to entry for robotics. A developer should be able to pick a "block" (code + diagram), connect the peripheral, and have a working ROS2 node in 5 minutes.

## 🏗 Block Architecture
Each example in this library is an atomic "Block" containing:
*   **ROS2 Node:** Clean, idiomatic code (Python/C++).
*   **Hardware Interface:** Hardware abstraction (GPIO, I2C, SPI, UART).
*   **Launch file:** Launch configuration with sane defaults.
*   **Documentation:**
    *   Wiring Diagram.
    *   Bill of Materials (BOM).
    *   Topic Interfaces (input/output).
    *   Verification Command (one-liner).

## 📂 Repository Structure
```text
xpi-blocks/
├── docs/               # Global documentation and guides
├── blocks/             # Main modules
│   ├── inputs/         # Input devices
│   │   ├── joystick/   # Gamepads (Xbox, PS4, Generic)
│   │   ├── rc_sbus/    # Radio Control (FrSky, ELRS via SBUS/CRSF)
│   │   ├── keyboard/   # Teleop wrappers
│   │   └── ...
│   ├── actuators/      # Actuators
│   │   ├── motors_dc/  # DC Motor drivers (L298N, BTS7960)
│   │   ├── servos/     # Servo control (Direct PWM, PCA9685)
│   │   ├── steppers/   # Stepper drivers (A4988, TMC2208)
│   │   └── relays/     # GPIO switching & Power distribution
│   └── sensors/        # Sensors
│       ├── imu/        # MPU6050, BNO055, etc.
│       ├── lidar/      # 2D Lidars (RPLidar, LD06)
│       ├── vision/     # RPi Cam, USB Cam, OpenCV wrappers
│       └── range/      # Distance sensors (HC-SR04, VL53L0X)
└── scripts/            # Setup utilities (Docker setup, udev rules)
```

## 🛠 Tech Stack & Standards
*   **Primary OS:** Ubuntu Server 22.04 LTS (64-bit) for Raspberry Pi.
*   **ROS2 Distro:** Humble Hawksbill (LTS).
*   **Compatibility:** Code is forward-compatible with Jazzy/Ubuntu 24.04 where possible.
*   **Language:** Python 3.10+ (primary), C++ (time-critical).
*   **GPIO Access:** `gpiozero` (Standardized HAL).
*   **Containerization:** `Dockerfile` support for rapid deployment.

## 📋 Prerequisites
Before using these blocks, ensure you have:
1.  **Raspberry Pi 4 or 5** (Zero 2W is also supported).
2.  **Ubuntu 22.04 Installed** (Official RPi Imager image).
3.  **ROS2 Humble Desktop/Base** installed and sourced.
    ```bash
    source /opt/ros/humble/setup.bash
    ```
4.  **GPIO Privileges:** User must be in `gpio` group (or `dialout` depending on OS).

## 📝 Documentation Guidelines
*   **README Driven:** Documentation is written before code.
*   **Visuals:** Photos of real wiring + Fritzing/Schematic diagrams.
*   **Troubleshooting:** Dedicated section for common issues in every block.