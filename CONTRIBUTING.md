# Contributing to XPI-Blocks

Thank you for your interest in contributing to XPI-Blocks! This project aims to simplify robotics development by providing modular, well-documented ROS2 blocks for Raspberry Pi.

To maintain high quality and consistency, we ask all contributors to follow these guidelines.

## 🌿 Branching Strategy

We use a standard branching model:
*   **main**: The stable branch. Represents the latest production-ready state.
*   **develop**: The integration branch for new features. All Pull Requests should be targeted here.
*   **feature/name**: For new device drivers or major features. Created from `develop`.
*   **fix/name**: For bug fixes.
*   **docs/name**: For documentation improvements.

## 📝 Commit Guidelines

We recommend using [Conventional Commits](https://www.conventionalcommits.org/):
*   `feat: add driver for BME680 sensor`
*   `fix: resolve I2C timeout in IP5306 node`
*   `docs: update wiring diagram for INA219`
*   `refactor: clean up GPIO handling in sonar node`

## 💻 Code Standards

### Python (ROS2 Nodes)
*   Follow **PEP 8** style guide.
*   Use `rclpy` best practices.
*   Avoid hardcoded values; use ROS2 parameters.
*   Always include clear logging (`self.get_logger().info(...)`).

### Documentation (The "Block" Standard)
Every new device driver must include a documentation folder in `blocks/<category>/<device_name>/` containing:
1.  **README.md**: Overview, wiring table, parameters, and topic interface.
2.  **Visuals**: Wiring diagrams or photos (if possible).

## 🚀 Pull Request Process

1.  **Fork the repository** and create your branch from `develop`.
2.  **Verify on Hardware**: If you are adding a driver, you *must* test it on a physical Raspberry Pi with the actual device.
3.  **Update Roadmap**: If adding a new device, move it from "Backlog" to "Implemented" in `ROADMAP.md`.
4.  **Submit PR**: Target your PR to the **develop** branch. Describe what you've added, provide a log of the node running successfully, and link any related issues.
5.  **Code Review**: At least one maintainer must review and approve your PR before merging into `develop`.
6.  **Release**: Periodically, `develop` will be merged into `main` for stable releases.

## 🛠 Development Environment

The easiest way to develop is using the provided **Dev Container** (VS Code) or the **Dockerfile**, which contains all ROS2 Humble dependencies.

```bash
# Build the dev image
docker compose build
```

## ⚖️ License

By contributing, you agree that your contributions will be licensed under the project's **MIT License**.
