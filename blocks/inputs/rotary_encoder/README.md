# Rotary Encoder (GPIO): Quadrature Decoder

This block reads standard **Incremental (Quadrature) Encoders**.
These are commonly found on DC motors (for odometry) or as user input dials.

The node counts "ticks" based on the state changes of pins A and B.

## ⚡ Wiring (GPIO)

| Encoder Pin | Raspberry Pi Pin | Description |
| :--- | :--- | :--- |
| **VCC** | 3.3V or 5V | Depends on encoder specs. |
| **GND** | GND | Ground. |
| **CLK / A** | GPIO X | Phase A. |
| **DT / B** | GPIO Y | Phase B. |
| **SW** | GPIO Z | (Optional) Push button. |

## 🚀 Usage

### 1. Launch the Node
```bash
ros2 launch xpi_inputs rotary_encoder.launch.py
```

### 2. Verify Data
```bash
ros2 topic echo /rotary_encoder/ticks
```
*Output: Integers incrementing/decrementing.*

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `pin_a` | int | `17` | GPIO pin for Phase A (BCM numbering). |
| `pin_b` | int | `27` | GPIO pin for Phase B. |
| `reverse` | bool | `False` | Invert counting direction. |
| `publish_rate` | float | `20.0` | Hz to publish tick count. |

## 🧩 Topics Interface

### Publishers
*   `~/ticks` (`std_msgs/msg/Int32`) - Cumulative tick count.
*   `~/velocity` (`std_msgs/msg/Float32`) - Speed in ticks/second.
