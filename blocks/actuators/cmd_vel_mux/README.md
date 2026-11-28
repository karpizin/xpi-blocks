# cmd_vel Mux and Emergency Stop

`cmd_vel_mux_node` decides **which source drives the robot**, and holds an **emergency stop**
that overrides all of them. Without it every teleop and autonomy node publishes to the same
`/cmd_vel`, the commands interleave, and the last one wins: a stop from the gamepad is
overwritten by the next message from the web page or the LLM agent.

```
RC receiver   ─► cmd_vel/rc      (100) ┐
gamepad       ─► cmd_vel/gamepad  (90) │
web joystick  ─► cmd_vel/web      (80) │               estop (Bool) ──┐
BLE           ─► cmd_vel/ble      (70) ├─► cmd_vel_mux_node ◄─────────┤ stop button (GPIO)
gestures      ─► cmd_vel/gesture  (60) │        │                     │
LLM agent     ─► cmd_vel/agent    (50) │        ▼                     └ ~/release_estop (service)
swarm         ─► cmd_vel/swarm    (40) ┘   cmd_vel_safe ─► diff_drive_node ─► motors
```

## 🧠 How it works
* The output follows the **highest-priority input that is active** (sent a command within its
  timeout). An operator holding the RC sticks still (zeros) still has control: presence wins.
* No input active → the output is **zero**, streamed at `publish_rate`.
* **Emergency stop latch.** `true` on `estop`, or the stop button, engages it: the output goes to
  zero at once and stays there whatever the inputs send. `false` does **not** release it; only
  the service does:
  ```bash
  ros2 service call /cmd_vel_mux_node/release_estop std_srvs/srv/Trigger
  ```
  The release is refused while the button is still pressed (or its wire is open).
* **Re-arming.** After a release each input is ignored until it sends a stop (a zero command)
  once, so a joystick held forward or an agent in the middle of a move cannot drive off the
  moment the stop is released.
* `~/estop_active` (Bool, latched) and `~/active_input` (String: `web`, `rc`, `none`, `estop`)
  report the state.

## 🔌 Stop button
A **normally closed** mushroom button between a GPIO and GND (`estop_gpio`, internal pull-up):
pressing it opens the circuit → stop. A cut wire or an unplugged button also reads as open, so
it stops the robot too, and the node starts latched if the circuit is open at startup. For a
normally open button set `estop_normally_closed:=false` (then a broken wire is *not* detected).

The button acts through software (this node and the motor drivers). For a robot that can hurt
someone, also cut the motor power with a hardware switch or relay.

## 🚀 Quick start
```bash
ros2 launch xpi_projects rover_tb6612.launch.py web:=true estop_gpio:=21
ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}"    # stop
```
Connect a source by remapping its `cmd_vel` to its mux input:
```bash
ros2 run xpi_inputs joy_mapper_node --ros-args -r cmd_vel:=cmd_vel/gamepad
ros2 run xpi_inputs rc_interpreter_node --ros-args -r cmd_vel:=cmd_vel/rc
ros2 run xpi_comms ble_bridge_node --ros-args -r /cmd_vel:=cmd_vel/ble
```

## ⚙️ Parameters
| Parameter | Default | Description |
|---|---|---|
| `inputs` | 7 inputs above | `'name:topic:priority:timeout'` strings; distinct priorities, higher wins. |
| `output_topic` | `cmd_vel_safe` | Twist output to the drive node. |
| `publish_rate` | `20.0` | Hz of the output stream. |
| `estop_topic` | `estop` | Bool; `true` engages the stop. |
| `estop_gpio` | `-1` | BCM pin of the stop button, `-1` = none. |
| `estop_normally_closed` | `true` | NC button: open circuit = stop. |
| `estop_on_start` | `false` | Start with the stop engaged; release it before driving. |
| `mock_hardware` | `false` | Simulated GPIO for the button. |
