# Mechanical Anemometer (Wind Speed Sensor)

A mechanical anemometer (cup anemometer) measures wind speed by counting the rotations of a set of cups. Inside the housing, a magnet typically passes a **reed switch** or a **Hall effect sensor**, closing a circuit once or twice per revolution.

## 🧠 Theory of Operation

The sensor outputs a series of pulses (interrupts). The frequency of these pulses (Pulses per Second, or Hz) is proportional to the wind speed.

**The Formula:**
`Wind Speed (m/s) = Frequency (Hz) * ConversionFactor`

**Standard Calibration:**
Many common hobbyist anemometers (like those in "Fine Offset" weather stations) follow this rule:
*   **1 Pulse per second (1 Hz) = 2.4 km/h = 0.666 m/s.**
*   Therefore, the default `factor` in this node is `0.666`.

## 📦 Bill of Materials
*   Raspberry Pi
*   Mechanical Anemometer (Cup type)
*   Jumper Wires

## 🔌 Wiring
Anemometers usually have two wires (it's a simple switch). 

| Anemometer Wire | Raspberry Pi | Note |
|-----------------|--------------|-----------------------------------|
| Wire 1          | GPIO 5       | Data Input (Default BCM 5 / Pin 29) |
| Wire 2          | GND          | Ground (Pin 6, 9, etc.) |

**Pull-up Resistor:** This node uses the Raspberry Pi's internal pull-up resistor. No external resistor is required.

## 🛠 Software Setup

This block uses `gpiozero`, which is part of the standard Raspberry Pi OS.

## 🚀 Usage

**Launch with default settings:**
```bash
ros2 launch xpi_sensors anemometer.launch.py
```

**Launch with custom calibration (e.g., if 1Hz = 1.0 m/s):**
```bash
ros2 launch xpi_sensors anemometer.launch.py factor:=1.0
```

## 📡 Interface

### Publishers
*   `~/wind_speed` (`std_msgs/Float32`): Current wind speed in **meters per second (m/s)**.

### Parameters
*   `gpio_pin` (int, default: `5`): BCM GPIO pin number.
*   `update_interval` (float, default: `5.0`): The calculation window in seconds. A longer interval provides more stable readings in gusty conditions.
*   `factor` (float, default: `0.666`): Conversion multiplier (m/s per Hz).
*   `mock_hardware` (bool, default: `false`): Run without real hardware for testing.

## 🔬 Advanced: Calibration
If you want to calibrate your anemometer manually:
1.  Drive a car at a constant speed (e.g., 20 km/h) on a calm day.
2.  Hold the anemometer outside (away from the car's slipstream).
3.  Note the frequency (Hz) reported by the node (using `ros2 node info` or debug logs).
4.  Calculate your factor: `Factor = CarSpeed_in_mps / Reported_Hz`.

## ⚠️ Troubleshooting
*   **Constant 0.0 reading:**
    *   Check wiring.
    *   Ensure the anemometer is actually spinning.
    *   Try connecting the GPIO pin directly to GND briefly to see if `pulse_count` increases.
*   **Erratic high readings:**
    *   **Debounce:** Mechanical switches can "bounce" (multiple signals per close). This node has a `0.01s` debounce time. If readings are too high, try increasing `bounce_time` in the source code.
    *   **EMI:** Long wires can act as antennas. Use shielded cable for long runs.
