# Testing in XPI-Blocks

This project uses `pytest` alongside standard ROS2 testing tools (`ament_cmake_pytest`).

## 🛠 Environment Setup

To run tests locally (even on macOS or Windows) without real hardware:

```bash
# Enable GPIO emulation
export GPIOZERO_PIN_FACTORY=mock
```

## 🧪 Running Tests

### Without ROS 2 (smoke tests)
Checks that every node starts in mock mode, that shipped parameter files are accepted and
that the hexapod math is right. Needs only Python 3.10+; also runs in CI:
```bash
pip install -r tests/requirements.txt
python -m pytest tests/ projects/hexapod_master/test -rs
```
See [tests/README.md](../tests/README.md) for what the ROS stub checks.
`tests/test_no_silent_mock.py` also starts every hardware node with `mock_hardware:=false` and no
hardware, and fails if a node switches to mock mode on its own or publishes anything.

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