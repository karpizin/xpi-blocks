# WS2812B (NeoPixel) Driver with Effects Library

This node controls addressable RGB LED strips (WS2812B, SK6812, etc.) and includes a built-in library of **100+ lighting effects** (Blink, Rainbow, Scanner, Fire, Matrix, etc.).

## 📦 Bill of Materials
*   Raspberry Pi
*   WS2812B LED Strip (NeoPixel) or Ring/Matrix.
*   **External 5V Power Supply** (Recommended for >8 LEDs).
*   Level Shifter (3.3V to 5V) or Diode trick (Optional but recommended).

## 🔌 Wiring
**GPIO 18 (PWM0)** is the standard pin for `rpi_ws281x`.

*   **VCC (5V)** <-> **External 5V+**
*   **GND** <-> **External GND** AND **Pi GND** (Common Ground is critical!)
*   **DIN (Data)** <-> **Pi GPIO 18** (Pin 12)

*Warning: Do not power long strips directly from the Pi's 5V pin. Each LED can draw 60mA. 30 LEDs = 1.8A!*

## 🚀 Quick Start
1.  **Install dependencies:**
    The `rpi_ws281x` library requires root privileges to access PWM.
    ```bash
    pip3 install rpi_ws281x
    ```
    *Note: The ROS2 node needs to run as root or with specific capabilities to access `/dev/mem`.*

2.  **Launch the driver:**
    ```bash
    # Must use sudo for hardware PWM access on RPi
    sudo -E env PATH=$PATH ros2 launch xpi_actuators ws2812_driver.launch.py led_count:=30
    ```
    *(The `-E env PATH=$PATH` trick helps preserve the ROS2 environment variables when using sudo)*

3.  **Test Effects:**
    ```bash
    # Turn on Red Solid
    ros2 topic pub --once /ws2812_driver/set_color std_msgs/msg/ColorRGBA "{r: 1.0, g: 0.0, b: 0.0, a: 1.0}"
    
    # Start Rainbow Cycle
    ros2 topic pub --once /ws2812_driver/set_effect std_msgs/msg/String "data: 'rainbow_cycle'"
    
    # Larson Scanner (Cylon)
    ros2 topic pub --once /ws2812_driver/set_effect std_msgs/msg/String "data: 'larson_scanner'"
    ```

## 📡 Interface
### Topics
| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `~/set_effect` | `std_msgs/String` | Input | Set effect name (e.g., `blink`, `fire`, `rainbow_cycle`). |
| `~/set_speed` | `std_msgs/Float32` | Input | Adjust effect speed (Hz or multiplier). |
| `~/set_color` | `std_msgs/ColorRGBA` | Input | Set base color for effects (Solid, Breathe, Wipe). |

### Parameters
*   `led_count` (int, default: `30`): Number of LEDs.
*   `led_pin` (int, default: `18`): GPIO pin.
*   `led_brightness` (int, default: `255`): Global brightness (0-255).
*   `initial_effect` (string, default: `solid`): Startup effect.
*   `update_rate` (float, default: `30.0`): Refresh rate in Hz.

## 🎨 Effects Library Status
The node includes a built-in library `xpi_actuators.lib.led_effects`. 
**[👉 Click here for the Full Effects Catalog & Parameters (EFFECTS_CATALOG.md)](EFFECTS_CATALOG.md)** to see details on how to use each effect.

### ✅ Group 1: Basic & Static
*   [x] **Solid Color** - Static color.
*   [x] **Blink** - On/Off blinking.
*   [x] **Breathe** - Smooth fading in/out.
*   [x] **Strobe** - Fast flashing.
*   [x] **Hyper Strobe** - Aggressive strobe.
*   [x] **Fade In**
*   [x] **Fade Out**
*   [x] **Alternating** - Even/Odd pixels.
*   [ ] **Multi-Blink**
*   [x] **Flash** - One-shot bright flash.

### 🌈 Group 2: Rainbows
*   [x] **Static Rainbow**
*   [x] **Rainbow Cycle** - Smooth flowing rainbow.
*   [x] **Rainbow Breathe** - Whole strip changes color.
*   [x] **Rainbow Strobe** - Fast rainbow flashes.
*   [x] **Rainbow Chase** - Running rainbow segments.
*   [x] **Glitter Rainbow** - Sparkling rainbow.
*   [ ] **Pastel Rainbow**
*   [ ] **Neon Rainbow**
*   [ ] **Vertical Rainbow**
*   [ ] **Double Rainbow**

### 🏃 Group 3: Chases & Scanners
*   [x] **Color Wipe** - Sequential filling.
*   [x] **Reverse Wipe** - Filling from end to start.
*   [x] **Wipe Random** - Filling with random colors.
*   [x] **Theater Chase** - Marching ants.
*   [ ] **Theater Chase Rainbow**
*   [x] **Larson Scanner** - Cylon/KITT eye.
*   [ ] **KITT Scanner**
*   [x] **Comet** - Single dot with fading trail.
*   [x] **Bounce** - Bouncing dot.
*   [x] **Dual Scan** - Two crossing scanners.
*   [x] **Train** - Block of pixels moving.
*   [ ] **Snake**
*   [ ] **Pac-Man**
*   [ ] **Conveyor Belt**
*   [x] **Marquee** - Theater border effect.

### ✨ Group 4: Sparkles & Weather
*   [x] **Sparkle** - Random white flashes.
*   [x] **Snow Sparkle** - Random flashes on background.
*   [ ] **Rain**
*   [ ] **Meteor Rain**
*   [x] **Fireflies** - Glowing and fading spots.
*   [x] **Twinkle** - Random pixel flickering.
*   [ ] **Twinkle Random**
*   [x] **Lightning** - Random storm flashes.
*   [ ] **Storm**
*   [ ] **Snowfall**
*   [ ] **Drizzle**
*   [ ] **Confetti**
*   [ ] **Popcorn**
*   [ ] **Explosion**
*   [ ] **Flicker (Candle)**

### 🔥 Group 5: Physics & Fluids
*   [x] **Fire** - Burning fire simulation.
*   [x] **Blue Fire** - Blue flame.
*   [x] **Ice Fire** - White/Cyan flame.
*   [ ] **Lava**
*   [x] **Water** - Flowing sine waves.
*   [ ] **Ripple**
*   [x] **Plasma** - Morphing color blobs.
*   [x] **Bubble** - Rising bubbles.
*   [ ] **Bouncing Balls**
*   [ ] **Multi-Ball**

### 🧘 Group 10: Meditative & Ambient (NEW)
*   [x] **Starry Night** - Twinkling stars on deep blue.
*   [x] **Firefly Field** - Moving yellow spots on green.
*   [x] **Fireplace** - Glowing embers.
*   [x] **Calm Ocean** - Slow morphing blues.
*   [x] **Sunny Forest** - Foliage with sunbeams.
*   [x] **Aurora Borealis** - Dancing polar lights.
*   [x] **Zen Pulse** - Ultra-slow breathing.
*   [x] **Morning Mist** - Drifting fog.
*   [x] **Autumn Leaves** - Golden drift on red.
*   [x] **Deep Space** - Glowing nebulas.

### 🥁 Group 11: Rhythmic & Music (BPM Sync)
*   [x] **20 BPM-based effects** (Beat Pulse, Bass Kick, Disco, etc.)

### 🎤 Group 12: Reactive (Audio Triggered)
*   [x] **Real-time Audio Reactive** (Flash, Burst, VU Level, etc.)

### 📊 Group 6: Utility
*   [x] **Progress Bar** - Linear fill based on `speed` (0-100).
*   [ ] **Battery Charge**
*   [ ] **Loading Spinner**
*   [ ] **Pulse Indicator**
*   [ ] **Traffic Light**
*   [x] **Police Lights** - Red/Blue strobe.
*   [ ] **Ambulance**
*   [ ] **Construction**
*   [ ] **Error Alert**
*   [ ] **Success**

### 🎨 Group 7: Palettes & Themes
*   [ ] **Cyberpunk**
*   [ ] **Halloween**
*   [ ] **Christmas**
*   [ ] **USA/Flag**
*   [ ] **Jungle**
*   [ ] **Ocean**
*   [ ] **Heatmap**
*   [ ] **Sunset**
*   [ ] **Zebra**
*   [ ] **Party**

### 📐 Group 8: Math & Audio
*   [ ] **Sine Wave**
*   [ ] **Cosine Interference**
*   [ ] **Perlin Noise**
*   [ ] **Simplex Noise**
*   [ ] **Sawtooth**
*   [ ] **Square Wave**
*   [ ] **VU Meter**
*   [ ] **Spectrum**
*   [ ] **Beat Detect**
*   [ ] **Dissolve**

### 👾 Group 9: Special
*   [ ] **Matrix (Digital Rain)**
*   [ ] **DNA**
*   [ ] **Tetris**
*   [ ] **Radar**
*   [ ] **Clock**
*   [ ] **Morse Code Message**
*   [ ] **TV Static**
*   [ ] **Fairy Dust**
*   [ ] **Heartbeat Sensor (ECG)**
*   [ ] **Off**

## ⚠️ Troubleshooting
*   **"Can't open /dev/mem":** You are not running as root. Use `sudo`.
*   **Colors are wrong (Green is Red):** WS2812 vs WS2811 vs GRB/RGB ordering. The library usually auto-detects, but might need tweaking in code (`strip_type` arg).
*   **Flickering:** Level shifter is needed, or the first LED is too far from the Pi (data signal degradation).
