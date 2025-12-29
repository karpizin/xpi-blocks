# Testing in XPI-Blocks

This project uses `pytest` alongside standard ROS2 testing tools (`ament_cmake_pytest`).

## 🛠 Environment Setup

To run tests locally (even on macOS or Windows) without real hardware:

```bash
# Enable GPIO emulation
export GPIOZERO_PIN_FACTORY=mock
```

## 🧪 Running Tests

### Via colcon (Recommended)
This command builds packages and runs all tests in the workspace:
```bash
colcon test --packages-select xpi_sensors
colcon test-result --verbose
```

### Directly via pytest
For rapid debugging of a specific file:
```bash
pytest src/xpi_sensors/test/test_sonar_node.py
```

## 🧩 Testing Principles

1.  **Use MagicMock**: We replace `sensor` or `bus` objects to test ROS message publication logic independently of the hardware.
2.  **Context Isolation**: Each test must initialize and shutdown its own ROS2 node context.
3.  **Type Validation**: Always verify message types and value ranges for published `sensor_msgs`.

Example Mock for I2C:
```python
from unittest.mock import MagicMock
mock_bus = MagicMock()
mock_bus.read_byte_data.return_value = 0x42
```