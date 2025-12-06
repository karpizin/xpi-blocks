# XPI-Blocks: Architectural Manifesto

This document defines the technical standards and architectural decisions for the `xpi-blocks` project.

## 1. Workspace & Package Organization
We will use a **Category-Based Package Structure** to keep dependencies clean. A user who only needs GPIO shouldn't need to install OpenCV.

```text
src/
├── xpi_interfaces/       # Custom .msg, .srv, .action files (keep minimal, prefer standard)
├── xpi_commons/          # Shared utilities, HAL base classes, mock definitions
├── xpi_inputs/           # Nodes for Joysticks, RC, Keyboards
├── xpi_sensors/          # Nodes for LiDARs, IMUs, Cameras
└── xpi_actuators/        # Nodes for Motors, Servos, Relays
```

## 2. Node Design Standard
We adopt a **Tiered Node Architecture**:

*   **Tier 1: Simple Blocks (Tutorials/Basic I/O)**
    *   Inherit from standard `rclpy.node.Node`.
    *   Focus: Readability and immediate results for beginners.
    *   *Example:* LED Blink, Button Press.

*   **Tier 2: Advanced Blocks (Drivers/Complex Logic)**
    *   Inherit from `rclpy.lifecycle.NodeLifecycle`.
    *   Focus: Safety, state management, and system integration.
    *   *Example:* Motor Controller (must stop on deactivate), IMU Driver.

## 3. Configuration & Parameters
*   **No Hardcoding:** All GPIO pins, I2C addresses, baud rates, and update frequencies must be declared as ROS Parameters.
*   **Parameter Descriptors:** All parameters must have a description, type, and (where applicable) min/max range defined in the code.
*   **YAML First:** Every block must include a `params.yaml` example.

## 4. Hardware Abstraction Layer (HAL)
To ensure compatibility across Raspberry Pi generations (4, 5, Zero) and allow easy mocking.

*   **Library:** We standardize on **`gpiozero`**.
    *   *Why?* It provides a clean, object-oriented API, excellent documentation, and built-in support for different pin factories (RPi.GPIO, lgpio, native), which is crucial for RPi 5 support.
*   **Interface Pattern:** Nodes should utilize `gpiozero` objects within their initialization.
*   **Mocking:** `gpiozero` supports "Mock Pin Factories". We will use this for CI/CD and laptop development tests.

## 5. Topics & Interfaces
*   **Standard Types:** Use `sensor_msgs`, `geometry_msgs`, `std_msgs` whenever possible.
    *   *Example:* A distance sensor must publish `sensor_msgs/Range`, not a custom `float32`.
*   **Namespacing:** All topics should be remappable, but default to generic names (e.g., `~/scan`, `~/cmd_vel`).

## 6. Dev Environment & CI
*   **Docker:** A `devcontainer` setup is mandatory to ensure reproducible builds across OS (macOS/Windows/Linux).
*   **Linting:** Strict `ament_flake8` and `ament_pep257` compliance.
