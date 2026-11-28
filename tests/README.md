# Tests without ROS 2

These tests run on any machine with Python 3.10+, no ROS 2 install and no hardware:

```bash
pip install -r tests/requirements.txt
pip install -r tests/requirements-optional.txt   # optional: covers more nodes
python -m pytest tests/ projects/hexapod_master/test -rs
```

They also run in CI (`smoke-test` job in `.github/workflows/ros2-ci.yml`).

## What is checked

| Test | What it catches |
| :--- | :--- |
| `test_node_smoke.py::test_node_starts` | Every node (all `console_scripts` in `src/*/setup.py` and `projects/*/nodes/*_node.py`) runs its `main()` in mock mode: start-up, a few timer rounds, shutdown. Fails on exceptions and on errors logged while handling a programming error (AttributeError, TypeError, wrong message field type, parameter type mismatch, ...). |
| `test_node_smoke.py::test_mock_node_publishes` | Nodes with a mock mode and a timer publish at least one message. |
| `test_config_params.py` | Every `src/*/config/*.yaml` parameter file is accepted by its node, as with `--params-file`. |
| `projects/hexapod_master/test` | Leg IK (checked against forward kinematics), body kinematics, interpolator, gait engine, gait node. |

A node whose third-party library is not installed is **skipped**, with the library name in the
reason (`-rs` prints them). Missing hardware (no serial port, OSError) is a skip as well, not a failure.

## The ROS stub (`tests/ros_stub/`)

The stub reproduces the parts of ROS 2 Humble where mistakes are easy to make, so a node that
fails here also fails on the robot:

* **Messages** are generated from the real `.msg`/`.srv` definitions (`msg_defs.json`) with the
  checks of `rosidl_generator_py`: unknown fields raise `AttributeError`, `Float32(data=5)` (int
  for a float) raises `AssertionError`, numeric arrays are numpy/array.array.
* **rclpy** exposes only the names the real modules have (`rclpy_api.json`), so
  `from rclpy.parameter import ParameterDescriptor` fails as it does on Humble.
* **Parameters** are statically typed like rclpy: a `[]` default is `BYTE_ARRAY`, a `None` default
  raises `ParameterUninitializedException` on `get_parameter()`, an override of another type raises
  `InvalidParameterTypeException`.
* `publish()` rejects the wrong message type, loggers accept only `str`, `Node.publishers` and
  friends are read-only properties, QoS is required, topic names are validated.
* Published messages are delivered in-process to subscriptions on the same topic.

To update the message definitions (e.g. for another ROS distribution), see the header of
`tests/ros_stub/gen_msg_defs.py`.

## Known issues

`KNOWN_ISSUES` in `test_node_smoke.py` lists nodes with a confirmed bug that needs a decision or
hardware to fix. They are strict `xfail`: once a node is fixed the test reports **XPASS** and fails,
so the entry must be removed.
