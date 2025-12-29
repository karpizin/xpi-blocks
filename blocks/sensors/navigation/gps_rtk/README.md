# RTK GPS Block (High Precision Positioning)

This block provides centimeter-level positioning accuracy using Real-Time Kinematic (RTK) technology.

## 🛰️ What is RTK and how does it differ from standard GPS?
Standard GPS receivers have an error margin of **2 to 5 meters** due to atmospheric distortions.
**RTK (Real-Time Kinematic)** uses an additional data stream (corrections) from a stationary **Base Station** with known coordinates. This eliminates errors and achieves an accuracy of **1-2 centimeters**.

### Fix Types:
1.  **NO FIX (0)**: No satellites found.
2.  **GPS FIX (1)**: Standard precision (2-5 meters).
3.  **FLOAT (2)**: Corrections received, but phase ambiguity is not yet resolved (~20-50 cm accuracy).
4.  **FIXED (3)**: Full phase resolution achieved. Centimeter precision (**1-3 cm**).

## 🔌 Connection
RTK modules (e.g., u-blox ZED-F9P) are typically connected via UART or USB.

## 🚀 How to activate RTK
To get centimeter-level accuracy, your receiver needs **RTCM corrections**.

### Method 1: NTRIP (via Internet)
1.  Register with a base station provider (e.g., RTK2GO or local networks).
2.  Run an NTRIP client on the Raspberry Pi to forward network data to the GPS serial port.

### Method 2: Local Base Station
One GPS module is set up as a static Base, and the second is on the robot (Rover). They communicate via a radio link (e.g., using our **HC-12** or **LoRa** blocks).

## 📡 ROS2 Interface
*   **Published Topic:** `~/fix` (`sensor_msgs/NavSatFix`)
*   **Status Topic:** `~/rtk_status` (Custom status including Fix Type)