# Differential Drive (cmd_vel → left/right motor)

`diff_drive_node` turns a body velocity command (`geometry_msgs/Twist` on `cmd_vel`) into the
speed commands of a two-wheeled robot's left and right motors. Without it, the teleop blocks
(web joystick, RC receiver, BLE, gestures, gamepad) publish `cmd_vel` that no motor driver
understands: the drivers take one `std_msgs/Float32` per motor.

```
web joystick / RC / BLE ...  ──Twist──►  cmd_vel_mux_node  ──cmd_vel_safe──►  diff_drive_node
                                                                  │
                                         ┌────────────────────────┴───────────────────────┐
                              /tb6612_driver_node/motor_a/cmd_vel        /tb6612_driver_node/motor_b/cmd_vel
```

## 🧠 How it works
* `left = (v - ω·L/2) / max_wheel_speed`, `right = (v + ω·L/2) / max_wheel_speed`, with `v` =
  `linear.x` (m/s), `ω` = `angular.z` (rad/s, positive = turn left), `L` = `wheel_separation`.
* If a wheel would need more than full duty, **both** wheels are scaled down by the same factor:
  the robot stays on the arc it was asked to drive, just slower.
* Commands are **streamed** at `publish_rate`, as the motor drivers' watchdogs require.
* No `cmd_vel` for `cmd_timeout` seconds → both motors 0. `NaN`/`inf` count as 0.
* `max_accel` (optional) ramps speeding up; slowing down and stopping are always immediate.

## 🚀 Quick start (TB6612, motor A = left, motor B = right)
```bash
ros2 run xpi_actuators tb6612_driver_node
ros2 launch xpi_actuators diff_drive.launch.py wheel_separation:=0.14 max_wheel_speed:=0.4
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```
Behind `cmd_vel_mux_node` (priorities, emergency stop) use `cmd_vel_topic:=cmd_vel_safe`.

Other drivers: set `left_topic`/`right_topic`, e.g. L298
`/l298_driver_node/motor_a/cmd_speed`. A motor that turns the wrong way: `invert_left:=true`.

## ⚙️ Parameters
| Parameter | Default | Description |
|---|---|---|
| `wheel_separation` | `0.15` | m between the wheel centers. |
| `max_wheel_speed` | `0.5` | m/s of a wheel at full duty (1.0). Measure it, or tune until `linear.x` matches. |
| `max_linear` | `0.0` | m/s limit on `linear.x`; 0 = only `max_wheel_speed` limits. |
| `max_angular` | `0.0` | rad/s limit on `angular.z`; 0 = none. |
| `max_accel` | `0.0` | duty per second when speeding up; 0 = no ramp. |
| `left_topic` / `right_topic` | TB6612 motor A / B | `std_msgs/Float32` motor command topics. |
| `invert_left` / `invert_right` | `false` | Reverse a motor. |
| `cmd_timeout` | `0.5` | s without `cmd_vel` before stopping. |
| `publish_rate` | `20.0` | Hz of the motor command stream. |

Open loop: without encoders `max_wheel_speed` is an estimate, and the real speed depends on
battery voltage and load.
