# Project: Smart Motor Unit (Closed-Loop)

This project combines a simple DC motor driver with sensors to create a **Smart Actuator**.
Instead of just "spinning blindly", this unit can:
1.  Measure its speed (Encoder).
2.  Measure its load/current (INA219).
3.  Detect stalls (High Current + Zero Speed).

## 🧩 Components
1.  **TB6612FNG** Driver (or L298N).
2.  **DC Motor** with Magnetic/Optical Encoder.
3.  **INA219** Current Sensor.
4.  **Raspberry Pi**.

## ⚡ Wiring Diagram
*(Conceptual)*
*   **Motor Driver** -> GPIOs (PWM/Dir).
*   **Encoder** -> GPIOs (A/B).
*   **INA219** -> I2C (SDA/SCL). connected in series with the Motor Power supply.

## 🚀 Usage
```bash
ros2 launch xpi_projects smart_motor.launch.py
```

## 📡 Interfaces
*   **Input:** `~/cmd_vel` (geometry_msgs/Twist) - Target speed.
*   **Output:** `~/encoder/velocity` - Actual speed.
*   **Output:** `~/power/battery_state` - Voltage/Current.
