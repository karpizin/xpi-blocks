# VNH2SP30 High-Power Motor Driver (Monster Moto Shield)

The VNH2SP30 is a high-current motor driver capable of delivering up to 30A (peak) per channel. It is commonly found in the "Monster Moto Shield" form factor.

## 🧠 Features
*   **High Power:** Suitable for large rovers and heavy robots.
*   **Current Sensing:** Provides an analog output proportional to motor current (can be read via ADS1115).
*   **Thermal Shutdown:** Built-in protection against overheating.

## 📦 Bill of Materials
*   Raspberry Pi
*   VNH2SP30 Motor Driver (e.g., Monster Moto Shield)
*   External Power Supply (7V - 16V)
*   High-power DC Motors

## 🔌 Wiring (Example for one motor)

| VNH2SP30 Pin | Raspberry Pi GPIO | Note |
|--------------|-------------------|---------------------------|
| PWM          | GPIO 12           | Speed Control |
| INA          | GPIO 5            | Direction A |
| INB          | GPIO 6            | Direction B |
| EN/DIAG      | GPIO 7 (Optional) | Enable / Fault Detect |
| CS           | ADC Pin           | Current Sense (Analog) |

## 🚀 Usage

**Launch the node:**
```bash
ros2 run xpi_actuators vnh2sp30_node
```

**Parameters:**
*   `motor_a_pwm_pin` (default: 12)
*   `motor_a_in1_pin` (default: 5)
*   `motor_a_in2_pin` (default: 6)
*   `motor_a_en_pin` (Optional)
*   `pwm_frequency` (default: 20000 Hz for silent operation)

## 📡 Interface

### Subscribers
*   `~/motor_a/cmd_speed` (`std_msgs/Float32`): -1.0 to 1.0.
*   `~/motor_b/cmd_speed` (`std_msgs/Float32`): -1.0 to 1.0.

### Current Monitoring
To monitor current, use the **Analog Sensor Interpreter** block with an **ADS1115** connected to the CS (Current Sense) pin of the VNH2SP30.
