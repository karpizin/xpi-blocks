# WS2812 Effects Catalog

This catalog details the visual behavior and parameter usage for each effect in the library.

**Usage:**
Send the **Effect ID** to the `~/set_effect` topic (String message).
Adjust parameters via `~/set_speed` (Float32) and `~/set_color` (ColorRGBA).

---

## 🟢 Group 1: Basic & Static
| Effect ID | Visual Description | Speed Param | Color Param | Status |
| :--- | :--- | :--- | :--- | :--- |
| `solid` | Sets the entire strip to a single static color. | Ignored | Sets the color. | ✅ |
| `blink` | Toggles the entire strip On/Off synchronously. | Frequency (Hz). Example: `1.0` = 1 blink/sec. | Sets the ON color. | ✅ |
| `breathe` | Smoothly fades brightness up and down (Sinusoidal). | Frequency (Hz) of full cycle. | Sets the base color. | ✅ |
| `strobe` | Fast, sharp flashes (10% duty cycle). | Frequency (Hz). Example: `10.0` | Sets the flash color. | ✅ |
| `hyper_strobe` | Extremely fast, aggressive strobe (Epilepsy Warning!). | Frequency (Hz). | Sets the flash color. | ✅ |
| `fade_in` | Smoothly transitions from Black to Color once. | Duration (s). | Target color. | ✅ |
| `fade_out` | Smoothly transitions from Color to Black once. | Duration (s). | Starting color. | ✅ |
| `alternating` | Odd pixels are Color 1, Even are Black. Swaps positions. | Swap frequency (Hz). | Color 1. | ✅ |
| `multi_blink` | Blinks Red, then Green, then Blue (or random sequence). | Frequency (Hz). | Ignored. | ❌ |
| `flash` | Single flash then fade to black. Triggered once. | Fade speed. | Flash color. | ✅ |

## 🌈 Group 2: Rainbows
| Effect ID | Visual Description | Speed Param | Color Param | Status |
| :--- | :--- | :--- | :--- | :--- |
| `static_rainbow` | Displays a rainbow gradient across the whole strip. | Ignored. | Ignored. | ✅ |
| `rainbow_cycle` | The rainbow gradient slowly moves/rotates along the strip. | Shift speed (Pixel steps per update or Hz). | Ignored. | ✅ |
| `rainbow_breathe`| The entire strip shows one color, which cycles through the rainbow. | Cycle speed (Hz). | Ignored. | ✅ |
| `rainbow_strobe` | Strobe effect, but each flash is a different rainbow color. | Frequency (Hz). | Ignored. | ✅ |
| `rainbow_chase` | Segments of rainbow colors "run" along the strip. | Movement speed. | Ignored. | ✅ |
| `glitter_rainbow`| `rainbow_cycle` with random white pixels flashing (glitter). | Speed of rainbow movement. | Ignored. | ✅ |
| `pastel_rainbow` | Softer, less saturated rainbow gradient. | Movement speed. | Ignored. | ❌ |
| `neon_rainbow` | High contrast, limited palette rainbow (Pink, Cyan, Lime). | Movement speed. | Ignored. | ❌ |
| `vertical_rainbow`| For matrices: Rainbow gradient flows vertically. | Movement speed. | Ignored. | ❌ |
| `double_rainbow` | Two rainbow gradients starting from ends and meeting in center. | Movement speed. | Ignored. | ❌ |

## 🏃 Group 3: Chases & Scanners
| Effect ID | Visual Description | Speed Param | Color Param | Status |
| :--- | :--- | :--- | :--- | :--- |
| `color_wipe` | Lights up pixels one by one until full, then clears/repeats. | Fill speed (pixels/sec or multiplier). | Fill color. | ✅ |
| `reverse_wipe` | `color_wipe` but from end to start. | Fill speed. | Fill color. | ✅ |
| `wipe_random` | `color_wipe` but with a random color each cycle. | Fill speed. | Ignored. | ✅ |
| `theater_chase` | "Marching ants" effect. Every 3rd pixel is on, pattern moves. | Movement speed. | Pattern color. | ✅ |
| `theater_chase_rainbow` | `theater_chase` but the pixels cycle rainbow colors. | Movement speed. | Ignored. | ❌ |
| `larson_scanner` | "KITT" / "Cylon" effect. A packet of light bounces back and forth with a fading trail. | Movement speed. | Scanner color (usually Red). | ✅ |
| `kitt_scanner` | Similar to Larson, but specifically mimics the Knight Rider car pattern. | Movement speed. | Scanner color. | ❌ |
| `comet` | A single dot with a long fading tail moving in one direction. | Speed. | Comet color. | ✅ |
| `bounce` | A simple dot bouncing off the edges (no tail). | Speed. | Dot color. | ✅ |
| `dual_scan` | Two Larson Scanners starting from ends and crossing in the middle. | Speed. | Scanner color. | ✅ |
| `train` | A solid block of pixels moving around the strip. | Speed. | Train color. | ✅ |
| `snake` | A snake that grows, moves, and shrinks (like the game). | Speed. | Snake color. | ❌ |
| `pac_man` | A yellow dot chased by a ghost dot. | Speed. | Ignored. | ❌ |
| `conveyor_belt` | Pattern shifts continuously. | Speed. | Pattern. | ❌ |
| `marquee` | Theater style border lights effect. | Speed. | Color. | ✅ |

## ✨ Group 4: Sparkles & Weather
| Effect ID | Visual Description | Speed Param | Color Param | Status |
| :--- | :--- | :--- | :--- | :--- |
| `sparkle` | Strip is black. Random pixels flash briefly. | Probability/Frequency of sparkles. | Sparkle color. | ✅ |
| `snow_sparkle` | Strip is Background Color. Random pixels flash White. | Probability/Frequency. | Background color (e.g., Gray). | ✅ |
| `rain` | Blue pixels "fall" from one end to the other, dimming out. | Rain speed. | Rain color (usually Blue). | ❌ |
| `meteor_rain` | Bright head, decaying trail, falling effect. | Speed. | Meteor color. | ❌ |
| `fireflies` | Pixels slowly fade in and out at random locations (soft sparkle). | Fade speed. | Firefly color (Yellow/Green). | ✅ |
| `twinkle` | Pixels flicker at different rates. | Flicker speed. | Base color. | ✅ |
| `twinkle_random` | `twinkle` with random colors. | Flicker speed. | Ignored. | ❌ |
| `lightning` | Entire strip or large sections flash randomly like lightning storms. | Storm intensity. | Flash color (White/Purple). | ✅ |
| `storm` | Dark pulsating background with occasional lightning flashes. | Pulse speed. | Background color. | ❌ |
| `snowfall` | Pixels fall and "stack up" at the bottom of the strip. | Fall speed. | Snow color (White). | ❌ |
| `drizzle` | Fast, short, dim droplets falling. | Speed. | Color. | ❌ |
| `confetti` | Random colored pixels appear and fade. | Density. | Ignored. | ❌ |
| `popcorn` | Pixels "jump" up from bottom and fall back. | Gravity/Speed. | Color. | ❌ |
| `explosion` | Bright flash in center expanding outwards and fading. | Expansion speed. | Explosion color. | ❌ |
| `flicker` | Simulates a candle flame (randomized brightness of Red/Orange). | Wind/Flicker amount. | Flame color. | ❌ |

## 🔥 Group 5: Physics & Fluids
| Effect ID | Visual Description | Speed Param | Color Param | Status |
| :--- | :--- | :--- | :--- | :--- |
| `fire` | Realistic fire simulation (Heat map algorithm). Hot at bottom, cooling up. | Cooling rate / Sparking chance. | Ignored (Always Red/Orange/Yellow). | ✅ |
| `blue_fire` | `fire` algorithm mapped to Blue/Cyan/White palette. | Cooling rate. | Ignored. | ✅ |
| `ice_fire` | `fire` algorithm mapped to White/Blue palette. | Cooling rate. | Ignored. | ✅ |
| `lava` | Slowly morphing red/black blobs. | Flow speed. | Ignored. | ❌ |
| `water` | Flowing waves of blue/aqua. | Flow speed. | Ignored. | ✅ |
| `ripple` | A wave (pulse) originates from center/random and spreads out. | Wave speed. | Water color. | ❌ |
| `plasma` | Psychedelic interference pattern of sine waves. | Animation speed. | Ignored. | ✅ |
| `bubble` | Individual pixels "bubble" up from bottom to top. | Rise speed. | Bubble color. | ✅ |
| `bouncing_balls` | Multiple dots falling and bouncing with gravity. | Gravity/Simulation speed. | Ball color. | ❌ |
| `multi_ball` | `bouncing_balls` but each ball is a different color. | Simulation speed. | Ignored. | ❌ |

## 📊 Group 6: Utility
| Effect ID | Visual Description | Speed Param | Color Param | Status |
| :--- | :--- | :--- | :--- | :--- |
| `progress_bar` | Lights up a percentage of the strip (0 to 100%). | **Acts as Percentage!** (0.0 to 100.0). | Bar color. | ✅ |
| `battery_charge` | `progress_bar` but changes color (Red < 20%, Yellow < 50%, Green > 50%). | **Acts as Percentage!** | Ignored. | ✅ |
| `loading_spinner` | A segment spins around (useful for Rings). | Spin speed. | Spinner color. | ✅ |
| `pulse_indicator` | A specific pixel (e.g., #0) pulses to show "alive" status. | Pulse speed. | Indicator color. | ❌ |
| `traffic_light` | Sets pixels to Red, Yellow, Green blocks. | Change speed (if animating). | Ignored. | ✅ |
| `police` | Alternates Red and Blue halves (or segments) of the strip. | Strobe speed. | Ignored. | ✅ |
| `ambulance` | Alternates Red and White. | Strobe speed. | Ignored. | ✅ |
| `construction` | Rotating or flashing Orange beacons. | Rotation speed. | Ignored. | ✅ |
| `error_alert` | Aggressive Red flashing/pulsing. | Flash speed. | Ignored (Red). | ✅ |
| `success` | A Green wipe or flash indicating success. Triggered once. | Animation speed. | Ignored (Green). | ✅ |

## 🎨 Group 7: Palettes & Themes
*(These apply static or slowly moving palettes to the strip)*
| Effect ID | Visual Description | Speed Param | Color Param | Status |
| :--- | :--- | :--- | :--- | :--- |
| `cyberpunk` | Pink and Cyan gradient. | Shift speed. | Ignored. | ✅ |
| `halloween` | Orange and Purple gradient. | Shift speed. | Ignored. | ✅ |
| `christmas` | Red and Green alternating pattern. | Shift speed. | Ignored. | ✅ |
| `usa_flag` | Red, White, Blue pattern. | Shift speed. | Ignored. | ❌ |
| `jungle` | Greens, Browns, Yellows. | Shift speed. | Ignored. | ❌ |
| `ocean` | Deep Blues, Cyans, Seafoam. | Shift speed. | Ignored. | ✅ |
| `heatmap` | Blue -> Green -> Yellow -> Red (Temperature map). | Shift speed. | Ignored. | ✅ |
| `sunset` | Purple -> Red -> Orange -> Yellow. | Shift speed. | Ignored. | ✅ |
| `zebra` | Black and White stripes. | Shift speed. | Ignored. | ❌ |
| `party` | Random assortment of vibrant colors. | Shift speed. | Ignored. | ✅ |

## 📐 Group 8: Math & Audio
| Effect ID | Visual Description | Speed Param | Color Param | Status |
| :--- | :--- | :--- | :--- | :--- |
| `sine_wave` | Brightness modulates via sine wave moving along strip. | Wave speed. | Base color. | ❌ |
| `cosine_interference` | Two waves interfering. | Speed. | Colors. | ❌ |
| `perlin_noise` | Smooth, organic random brightness/color changes (Cloud effect). | Evolution speed. | Base hue. | ❌ |
| `simplex_noise` | Similar to Perlin but computationally different/faster look. | Speed. | Base hue. | ❌ |
| `sawtooth` | Brightness ramps up and drops sharply. | Speed. | Color. | ❌ |
| `square_wave` | Hard on/off blocks moving. | Speed. | Color. | ❌ |
| `vu_meter` | Reacts to audio volume (Requires audio input source). | Sensitivity. | Ignored. | ❌ |
| `spectrum` | Audio frequency spectrum (Requires FFT). | Sensitivity. | Ignored. | ❌ |
| `beat_detect` | Flashes on bass beat. | Sensitivity. | Flash color. | ❌ |
| `dissolve` | Pixels randomly turn black until empty. | Decay speed. | Ignored. | ❌ |

## 👾 Group 9: Special
| Effect ID | Visual Description | Speed Param | Color Param | Status |
| :--- | :--- | :--- | :--- | :--- |
| `matrix` | Green "code" falling down (Matrix digital rain). | Rain speed. | Ignored (Green). | ✅ |
| `dna` | Two sine waves of different colors intertwining. | Rotation speed. | Colors. | ✅ |
| `tetris` | Blocks fall and stack up. | Fall speed. | Block colors. | ❌ |
| `radar` | (For Rings) A fading beam rotates like a radar sweep. | Sweep speed. | Beam color. | ✅ |
| `clock` | (For Rings) Displays hours/minutes/seconds. | Ignored. | Hand colors. | ❌ |
| `morse_code` | Flashes a text message (hardcoded or param) in Morse. | WPM speed. | Flash color. | ✅ |
| `tv_static` | Random black/white/gray noise. | Noise speed. | Ignored. | ✅ |
| `fairy_dust` | Sparkles that leave a faint trail. | Speed. | Dust color. | ❌ |
| `heartbeat` | A blip that travels and pulses like an ECG. | Pulse rate (BPM). | Line color. | ✅ |
| `off` | Turns all pixels OFF (Black). | Ignored. | Ignored. | ✅ |

## 🚨 Group 13: Indication (USIS Patterns)
| Effect ID | Visual Description | Speed Param | Color Param | Status |
| :--- | :--- | :--- | :--- | :--- |
| `double_blink` | Two flashes followed by a pause. | Frequency. | Warning color. | ✅ |
| `sos` | Morse code S-O-S signal. | Speed multiplier. | Alert color. | ✅ |
| `fast_blink` | High-frequency blinking (5Hz+). | Frequency. | Signal color. | ✅ |
| `status_pulse` | Short periodic "blip" (heartbeat). | Interval. | Status color. | ✅ |


## 🧘 Group 10: Meditative & Ambient
| Effect ID | Visual Description | Speed Param | Color Param | Status |
| :--- | :--- | :--- | :--- | :--- |
| `starry_night` | Deep blue with twinkling white/blue stars. | Twinkle frequency. | Ignored. | ✅ |
| `firefly_field` | Dark green with slow rising yellow flies. | Spawn density. | Ignored. | ✅ |
| `fireplace` | Pulsating red and orange embers. | Flicker intensity. | Ignored. | ✅ |
| `calm_ocean` | Slow morphing of blue and seafoam colors. | Morph speed. | Ignored. | ✅ |
| `sunny_forest` | Green foliage with golden sunbeam glints. | Beam movement. | Ignored. | ✅ |
| `aurora_borealis` | Dancing emerald, purple and blue waves. | Dancing speed. | Ignored. | ✅ |
| `zen_pulse` | Ultra-slow breathing of warm white/amber. | Pulse speed. | Base color. | ✅ |
| `morning_mist` | Drifting grey-blue and white misty clouds. | Drift speed. | Ignored. | ✅ |
| `autumn_leaves` | Dark red carpet with occasional falling gold leaves. | Leaf frequency. | Ignored. | ✅ |
| `deep_space` | Black void with occasional glowing nebulas. | Nebula evolution. | Ignored. | ✅ |

## 🥁 Group 11: Rhythmic & Music (BPM Sync)
*Note: In this group, the **Speed** parameter acts as **BPM**.*

| Effect ID | Visual Description | Speed Param (BPM) | Color Param | Status |
| :--- | :--- | :--- | :--- | :--- |
| `beat_pulse` | Smooth breathing in sync with beat. | BPM. | Base color. | ✅ |
| `beat_blink` | Hard on/off toggle on every beat. | BPM. | Base color. | ✅ |
| `beat_strobe` | Very sharp flash on the beat. | BPM. | Flash color. | ✅ |
| `rhythmic_chase` | Pattern shifts by 1 pixel every beat. | BPM. | Pattern color. | ✅ |
| `bass_kick` | Red flash on beat 1, blue on others. | BPM. | Ignored. | ✅ |
| `metronome` | Single pixel bounces left-to-right over 4 beats. | BPM. | Pixel color. | ✅ |
| `center_out_beat` | Light expands from center on every beat. | BPM. | Color. | ✅ |
| `rhythmic_rainbow` | Rainbow shifts in steps every beat. | BPM. | Ignored. | ✅ |
| `side_to_side` | Block travels end-to-end every beat. | BPM. | Color. | ✅ |
| `rhythmic_alternating` | Swaps odd/even pixels every beat. | BPM. | Color. | ✅ |
| `disco_strobe` | Random color flash on every beat. | BPM. | Ignored. | ✅ |
| `heartbeat_bpm` | Realistic ECG double-pulse. | BPM. | Line color. | ✅ |
| `bpm_scanner` | Scanner hits the edge exactly on the beat. | BPM. | Scanner color. | ✅ |
| `rhythmic_fade` | Lights up on beat and fades out until next. | BPM. | Color. | ✅ |
| `energy_bar` | Fills up over a 4-beat measure. | BPM. | Color. | ✅ |
| `rhythmic_glitter` | Burst of sparkles on every beat. | BPM. | Ignored. | ✅ |
| `color_switch` | Changes whole strip to random color on beat. | BPM. | Ignored. | ✅ |
| `bpm_snake` | Snake travels its length every beat. | BPM. | Color. | ✅ |
| `vu_meter_beat` | Simulated VU level jump on every beat. | BPM. | Color. | ✅ |
| `rhythmic_bounce` | Parabolic bounce hitting "floor" on beat. | BPM. | Color. | ✅ |

## 🎤 Group 12: Reactive (Audio Triggered)
*Note: These effects require an active **Beat Detector Node** publishing to `/audio/beat`.*

| Effect ID | Visual Description | Speed Param | Color Param | Status |
| :--- | :--- | :--- | :--- | :--- |
| `reactive_flash` | Full strip flash on beat, fading out. | Decay speed. | Flash color. | ✅ |
| `reactive_random_color` | Flash with random color on every beat. | Decay speed. | Ignored. | ✅ |
| `reactive_burst` | Light expands from center on every beat. | Expansion speed. | Color. | ✅ |
| `reactive_sparkle` | Explosion of random sparks on beat. | Decay speed. | Sparkle color. | ✅ |
| `reactive_vu` | Level bar jumping based on beat intensity. | Decay speed. | Bar color. | ✅ |
