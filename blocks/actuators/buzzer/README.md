# Actuator: Buzzer & Melodies (GPIO/PWM)

This block allows the robot to emit audible signals, from simple status beeps to complex RTTTL (Nokia-style) melodies.

## 📌 Features
*   **Simple Mode:** Beep on/off via `std_msgs/Bool`.
*   **Melody Mode:** Play RTTTL strings via `std_msgs/String`.
*   **Built-in Library:** Presets for common sounds (Startup, Error, Success).

## 🔌 Wiring Diagram

### 1. Active Buzzer (Simplest)
Active buzzers have an internal oscillator and only need a DC voltage to sound.
| Buzzer Pin | Raspberry Pi Pin | Note |
| :--- | :--- | :--- |
| VCC (+) | GPIO 18 (Pin 12) | Use a transistor for high current if needed |
| GND (-) | GND | |

### 2. Passive Buzzer (Magnetic/Piezo)
Passive buzzers require a PWM signal to produce different frequencies.
| Buzzer Pin | Raspberry Pi Pin | Note |
| :--- | :--- | :--- |
| Signal (+) | GPIO 18 (Pin 12) | Must be a PWM-capable pin |
| GND (-) | GND | |

## 🚀 Quick Start

1.  **Run the Node:**
    ```bash
    ros2 launch xpi_actuators buzzer.launch.py
    ```
2.  **Make a Simple Beep:**
    ```bash
    ros2 topic pub --once /buzzer/beep std_msgs/Bool "data: true"
    ```
3.  **Play a Melody (RTTTL):**
    ```bash
    ros2 topic pub --once /buzzer/melody std_msgs/String "data: 'StarWars:d=4,o=5,b=45:32p,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,4f#.6,32e6,32d#6,32c#6,8b.6,4f#.6,32e6,32d#6,32e6,8c#.6'"
    ```

## 📊 Topics
*   `~/beep` ([std_msgs/Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html)): Toggle buzzer sound (for active buzzers or fixed-tone passives).
*   `~/melody` ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)): RTTTL string to play.
