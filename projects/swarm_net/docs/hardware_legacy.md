# Recommended Hardware for Meshtastic LoRa Mesh

This document provides an overview of hardware compatible with the `Meshtastic Swarm` module, divided by usage type: for robot onboard systems, ground stations, and integration with Raspberry Pi.

---

## 1. Standalone Modules
The best choice for integration via USB or creating mobile communication nodes.

### 1.1 Heltec WiFi LoRa 32 (V3)
*   **Chip:** ESP32-S3 + Semtech SX1262.
*   **Pros:** 0.96" OLED screen (convenient for on-site diagnostics), low price, high availability.
*   **Usage:** Ground stations, debug nodes.
*   **Interface:** USB-C (Serial).

### 1.2 LilyGO T-Beam (V1.1 / V1.2)
*   **Chip:** ESP32 + SX1276/SX1262 + GPS (Neo-6M/M8N).
*   **Pros:** Integrated GPS and 18650 battery holder.
*   **Usage:** Full-featured "all-in-one" mobile node.
*   **Cons:** Relatively heavy for installation on small drones.

### 1.3 RAK Wireless WisBlock (RAK4631)
*   **Chip:** nRF52840 (Bluetooth 5.0) + SX1262.
*   **Pros:** **Extremely low power consumption**. Modular design (sensors or other interfaces can be added).
*   **Usage:** Ideal for drone onboard systems where weight and battery life are important.
*   **Interface:** UART / USB.

---

## 2. Expansion Modules (Hats) for Raspberry Pi
If the robot is controlled by a Raspberry Pi, it is more convenient to use Hats for direct connection to GPIO.

### 2.1 Waveshare SX1262 LoRa HAT
*   **Pros:** Direct connection to 40-pin GPIO. Good documentation and support for 868/915 MHz frequencies.
*   **Interface:** SPI (in Meshtastic, custom build setup or UART bridge usage might be required).
*   **Note:** Most off-the-shelf Hats from Waveshare use the LoRaWAN/Raw LoRa protocol. To work with Meshtastic, you need to ensure firmware compatibility.

### 2.2 RAK Wireless RAK2245 / RAK5146 (Concentrators)
*   **Description:** These are powerful gateways.
*   **Usage:** To create a powerful base station capable of listening to dozens of channels simultaneously. Not suitable for a drone onboard module.

---

## 3. Recommendations for Drone Swarms

To minimize weight and maximize range, it is recommended to:

1.  **Module Choice:** **RAK4631** (WisBlock Core) — weighs a few grams, runs on 3.3V, almost no heat.
2.  **Antenna:**
    *   For the drone: Flexible dipole antenna ("rubber ducky") or lightweight PCB antenna.
    *   For the base: Collinear antenna 5.8 dBi - 8 dBi, mounted on a mast.
3.  **Power:** Connect directly to the drone's BEC (Voltage Regulator) at 3.3V or 5V (depending on the module).

---

## 4. Wiring Diagram

### Option A: USB (Simplest)
*   Module (Heltec/T-Beam) -> USB-C cable -> Robot USB port.
*   The driver will detect the device as `/dev/ttyUSB0` or `/dev/ttyACM0`.

### Option B: UART (For direct integration)
If there are no free USB ports on the controller:
*   **Module TX** -> **Robot RX**
*   **Module RX** -> **Robot TX**
*   **GND** -> **GND**
*   **VCC** -> **3.3V/5V**

---

## 5. Frequency Selection (Regions)
*   **Europe / Russia:** 868 MHz (primary), 433 MHz (secondary).
*   **USA / Canada:** 915 MHz.
*   **Asia:** 433 MHz / 923 MHz.

*Note: Before purchasing, ensure that all modules in your swarm operate on the same frequency.*