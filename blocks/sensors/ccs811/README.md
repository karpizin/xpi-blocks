# CCS811: Air Quality Sensor (eCO2 & TVOC)

The **CCS811** is a digital gas sensor solution for monitoring indoor air quality. It detects:
1.  **eCO2** (Equivalent Carbon Dioxide): 400ppm to 8192ppm.
2.  **TVOC** (Total Volatile Organic Compounds): 0ppb to 1187ppb.

It uses a MOX (Metal Oxide) sensor and an onboard microcontroller to calculate these values.

**⚠️ Warm-Up Time:** The sensor needs to run for 20 minutes to give stable readings, and up to 48 hours of "burn-in" when new.

## ⚡ Wiring (I2C)

| CCS811 Pin | Raspberry Pi Pin | Description          |
| :---       | :---             | :---                 |
| **VCC**    | 3.3V (Pin 1)     | Power (1.8V - 3.3V)  |
| **GND**    | GND (Pin 6)      | Ground               |
| **SDA**    | GPIO 2 (Pin 3)   | I2C Data             |
| **SCL**    | GPIO 3 (Pin 5)   | I2C Clock            |
| **WAKE**   | GND              | Must be tied to GND! |

> **Crucial:** Connect the **WAKE** pin to GND, otherwise the sensor stays in sleep mode and won't respond to I2C.

## 📦 Bill of Materials
*   1x CCS811 Breakout Board (CJMCU-811 or Adafruit)
*   5x Jumper Wires

## 🚀 Usage

### 1. Launch the Node
```bash
ros2 launch xpi_sensors ccs811.launch.py
```

### 2. Verify Data
```bash
ros2 topic echo /ccs811/data
```
*Output: `[eCO2, TVOC]`.*

## ⚙️ ROS2 Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `polling_rate` | float | `1.0` | Hz. Don't poll too fast (1Hz is optimal). |

## 🧩 Topics Interface

### Publishers
*   `~/data` (`std_msgs/msg/Float32MultiArray`)
    *   Layout: `[eCO2 (ppm), TVOC (ppb)]`
*   `~/eco2` (`std_msgs/msg/Int32`) - eCO2 only.
*   `~/tvoc` (`std_msgs/msg/Int32`) - TVOC only.
