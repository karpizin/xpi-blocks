# Hardware Selection Guide for Meshtastic

This document provides recommendations for selecting hardware to integrate LoRa Mesh into XPI robotics systems, with a focus on swarm systems and autonomous repeaters.

---

## 1. Integration Scenarios

### 1.1 Onboard Computer (Raspberry Pi / Jetson)
If the robot has a powerful host processor, the LoRa module is used as a peripheral modem.

*   **Recommended Solution:** **RAK Wireless WisBlock (RAK4631)**.
    *   **Advantages:** The nRF52840 chip consumes 3-5 times less power than the ESP32. It produces minimal radio interference for drone GPS receivers.
    *   **Interface:** USB (via RAK5005-O base board) or UART (direct GPIO connection).
*   **Alternative (HAT):** **Waveshare SX1262 LoRa HAT**.
    *   **Advantages:** Compact "sandwich" installation.
    *   **Complexity:** Requires specific firmware configuration to work as a Meshtastic Node under Linux.

### 1.2 Standalone Node (Micro-drones / Beacons)
When weight and size are critical and flight controller (MCU) computing power is sufficient.

*   **Recommended Solution:** **Heltec LoRa Stick (V3)**.
    *   USB flash drive sized, minimal weight.
*   **LilyGO T-Beam S3 Supreme:**
    *   Built-in high-precision GPS and battery charge controller.

---

## 2. Special Solution: Drop-and-Forget Repeater

Scenario: A drone delivers a repeater to a hard-to-reach point (building roof, hilltop), drops it, and the node starts working autonomously to expand the swarm's coverage.

### 2.1 Technical Requirements
1.  **Energy Efficiency:** Must work for weeks without recharging.
2.  **Impact Resistance:** Must survive a 2-5 meter drop.
3.  **Self-sufficiency:** Automatic startup and neighbor discovery.

### 2.2 Recommended BOM
*   **Controller:** **RAK4631 (WisBlock Core)**. Using nRF52 is the only way to achieve real autonomy.
*   **Power:**
    *   Battery: Li-Po or Li-ion 18650 (3000-3500 mAh).
    *   Solar Panel: Small 5V/100mA panel mounted on the top of the case. RAK WisBlock has a built-in solar charge controller.
*   **Antenna:** Short flexible 868 MHz antenna ("Stubby") to prevent breaking upon impact with the ground.

### 2.3 Design Features
*   **Shock Absorption:** The case should be printed from TPU (flexible plastic) or have rubber dampers.
*   **Orientation:** Weighted bottom (battery at the bottom) so the module lands antenna-up.
*   **Sealing:** IP65/67 rating for protection against rain and dew.

### 2.4 Software Logic for Dropped Repeater
*   **"Router" or "Repeater" Mode:** In Meshtastic settings, the node should be marked as a static repeater (this saves airtime as it doesn't send packets about its movements).
*   **Deep Sleep:** Configure the node to enter sleep mode on low battery and wake up when the sun appears.

---

## 3. Antenna Strategy
Swarm efficiency depends 70% on antennas, not transmitter power.

*   **For Flying Nodes:** Dipole antennas on a flexible cable (U.FL -> SMA). It is important to move the antenna as far as possible from motor ESCs and video transmitters.
*   **For Ground Stations:** Collinear antennas (Omni-directional) with 5.8 dBi gain. Mounting on a mast increases range by 2-3x.

---

## 4. Selection Summary Table

| Task | Recommended Hardware | Interface | Priority |
| :--- | :--- | :--- | :--- |
| **Control Center (RPi)** | RAK4631 (WisBlock) | USB/UART | Energy Efficiency |
| **Swarm Drone** | Heltec V3 Stick | UART | Weight / Size |
| **Dropped Repeater** | RAK4631 + Solar | Autonomous | Autonomy |
| **Mobile Terminal** | LilyGO T-Echo | Bluetooth/BLE | E-Ink Screen |
