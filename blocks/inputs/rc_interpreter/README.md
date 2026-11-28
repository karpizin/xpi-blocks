# RC Interpreter

The RC Interpreter block converts raw RC channel arrays into higher-level robot
commands such as `geometry_msgs/Twist` and discrete action topics. It is
designed to sit after SBUS, CRSF, or PPM receiver nodes and normalize channel
behavior with deadzone, expo, scaling, and optional mode switches.

## Typical Use

Use this block when you already have a receiver node publishing channel data and
you want stable semantic control outputs for driving, steering, camera pan/tilt,
or mode selection.

## Key Capabilities

* Map throttle and steering channels into `Twist`
* Apply deadzone and exponential response curves
* Convert auxiliary channels into booleans or discrete states
* Keep downstream control nodes independent from receiver protocol details

## Implementation

The current implementation lives in:

* `src/xpi_inputs/xpi_inputs/rc_interpreter_node.py`

## Related Blocks

* `blocks/inputs/rc_sbus/README.md`
* `blocks/inputs/rc_crsf/README.md`
* `blocks/inputs/rc_ppm/README.md`
* `blocks/inputs/joy_mapper/README.md`
