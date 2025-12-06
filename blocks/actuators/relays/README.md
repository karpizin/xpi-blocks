# Relay / LED Block

This block demonstrates how to control a simple digital output device like a Relay Module or an LED using ROS2.

## 📦 Bill of Materials
*   Raspberry Pi (Any model with 40-pin header)
*   Relay Module (1-channel or similar) OR LED + 220Ω Resistor
*   Jumper Wires (Female-Female)

## 🔌 Wiring
| Relay Module | Raspberry Pi | Note |
|Data | GPIO 17 (Pin 11)| Configurable |
| VCC | 5V (Pin 2) | Check your relay voltage! |
| GND | GND (Pin 6) | |

## 🚀 Quick Start
Run the node (using internal `mock` mode if you don't have hardware connected yet):

```bash
ros2 launch xpi_actuators relay.launch.py mock_hardware:=true
```

## 📡 Interface
### Subscribers
*   `~/cmd` (`std_msgs/Bool`): Send `True` to turn ON, `False` to turn OFF.

### Parameters
*   `gpio_pin` (int, default: 17): BCM pin number.
*   `active_high` (bool, default: true): Set `false` for "Low Level Trigger" relays.

## ✅ Verification
To toggle the relay, open a new terminal and run:

```bash
ros2 topic pub --once /my_relay/cmd std_msgs/msg/Bool "{data: true}"
```
