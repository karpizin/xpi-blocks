# TSL2591 High Dynamic Range Light Sensor

The **TSL2591** is a very high sensitivity light-to-digital converter that transforms light intensity into a digital signal output capable of direct I2C interface. It is significantly better than standard CdS cells or simple photodiodes because it offers a huge dynamic range (188uLux to 88,000 Lux) and separates Infrared light from Visible light.

## 🧠 Theory of Operation

The TSL2591 has two photodiodes:
1.  **Channel 0 (Full Spectrum):** Measures Visible light + Infrared light.
2.  **Channel 1 (Infrared):** Measures Infrared light only.

**Why is this important?**
Human eyes don't see Infrared, but silicon sensors do. To get a "Lux" value that matches human perception, we must subtract the IR component.
`Visible Light = Channel 0 - Channel 1`

The driver automatically performs this calculation to output accurate **Lux**.

## 📦 Bill of Materials
*   Raspberry Pi
*   TSL2591 Module (I2C) - [Adafruit](https://www.adafruit.com/product/1980) or generic.
*   Jumper Wires

## 🔌 Wiring
Connect the TSL2591 module to the I2C pins.

| TSL2591 Pin | Raspberry Pi | Note |
|-------------|--------------|-----------------------------------|
| VIN         | 3.3V or 5V   | Power (Pin 1).                    |
| GND         | GND          | Ground (Pin 6)                    |
| SDA         | GPIO 2 (SDA) | Data (Pin 3)                      |
| SCL         | GPIO 3 (SCL) | Clock (Pin 5)                     |

**I2C Address:** 0x29 (Fixed).

## 🛠 Software Setup

1.  **Install System Dependencies:**
    ```bash
    sudo apt-get install python3-libgpiod
    ```

2.  **Install Python Library:**
    ```bash
    pip3 install adafruit-circuitpython-tsl2591
    ```

## 🚀 Usage

**Launch with defaults:**
```bash
ros2 launch xpi_sensors tsl2591.launch.py
```

## ⚙️ Configuration Guide: Gain & Timing

To get the best readings, you need to balance **Gain** (Sensitivity) and **Integration Time** (Shutter speed).

### 1. Gain (Sensitivity)
Sets how much the electrical signal is amplified.

| Setting | Multiplier | Best Use Case |
| :--- | :--- | :--- |
| **LOW** | 1x | Bright Sunlight, Outdoors. Prevents saturation. |
| **MEDIUM** | 25x | **Default**. Standard Indoor lighting. |
| **HIGH** | 428x | Dim Indoor light, Hallways at night. |
| **MAX** | 9876x | Pitch black rooms, Starlight, Night vision. |

### 2. Integration Time (Shutter Speed)
Sets how long the sensor "collects" light before reading.

| Setting | Time | Speed | Description |
| :--- | :--- | :--- | :--- |
| **100MS** | 100ms | Fast | **Default**. Good for changing light. |
| **...** | ... | ... | Steps of 100ms up to 600ms. |
| **600MS** | 600ms | Slow | Collects 6x more light than 100ms. Good for low light stability. |

**Rule of Thumb:**
*   **Too Bright (Output 0 or Saturation)?** -> Decrease Gain, Decrease Time.
*   **Too Dark (Noise)?** -> Increase Gain, Increase Time.

## 📡 Interface

### Publishers
*   `~/illuminance` (`sensor_msgs/Illuminance`): Calculated Lux (Human perception).
*   `~/infrared_raw` (`std_msgs/Float32`): Raw 16-bit count from IR channel. 
    *   *High values indicate strong heat sources or sunlight.*

### Parameters
*   `polling_rate` (float, default: `2.0`): Hz.
*   `gain` (string, default: `MEDIUM`): `LOW`, `MEDIUM`, `HIGH`, `MAX`.
*   `integration_time` (string, default: `100MS`): `100MS` to `600MS`.
*   `frame_id` (string, default: `tsl2591_link`): Frame ID for TF.

## ⚠️ Troubleshooting
*   **"Lux 0.0" when it's bright**: 
    *   **Saturation!** The sensor is overwhelmed. Switch Gain to `LOW`.
    *   Raw counts for Ch0 or Ch1 are hitting 65535 (16-bit limit).
*   **"Lux is erratic"**:
    *   Fluorescent lights flicker at 50/60Hz. Try using integration times that are multiples of 100ms (100ms, 200ms) to average out the flicker.