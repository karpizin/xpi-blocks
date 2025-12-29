# Power Calculation for Autonomous Repeater

Power system parameter calculations to ensure continuous (24/7) operation of a RAK4631-based node for 12+ months.

## 1. Consumption (Energy Expenditure)
*   **Base Current (Idle/Listen):** 10 mA.
*   **Weighted Average Current (including transmissions):** 15 mA.
*   **System Voltage:** 3.7 V.
*   **Daily Consumption:** 15 mA * 24 h = **360 mAh**.
*   **Considering Conversion Efficiency (80%):** 360 / 0.8 = **450 mAh/day**.

## 2. Battery (Reserve)
To survive 10 days of total cloud cover:
*   **Required Capacity:** 450 mAh * 10 days = 4500 mAh.
*   **Selection:** 1x Li-ion 21700 (5000 mAh) or 2x 18650 (3000 mAh each).
*   **Feature:** Excess capacity also compensates for chemistry degradation at low temperatures.

## 3. Solar Panel (Generation)
*   **Average Effective Insolation (winter):** 2-3 hours.
*   **Required Charging Current:** 450 mAh / 2 h = 225 mA.
*   **Panel Power (P = UI):** 5 V * 0.225 A = 1.125 W.
*   **Safety Factor (orientation, dust, wear):** 2.0x.
*   **Final Panel Power:** **2.25 W — 3 W**.

## 4. Dimensions and Physical Characteristics of Panels
Modern monocrystalline panels (20-22% efficiency) have the following approximate sizes:

| Power | Approximate Size | Weight | Optimal Use Case |
| :--- | :--- | :--- | :--- |
| **3 W** | 120 x 150 mm | ~70 g | Main standard for droppable nodes |
| **5 W** | 150 x 200 mm | ~110 g | Large nodes, base stations |

### Placement Recommendations:
1.  **"Sail" Effect:** A 5W panel has a large area (300 cm²), increasing the risk of being tipped over by wind during a drop.
2.  **Redundancy:** To increase survivability, it is recommended to use **two 1.5 - 2 W panels** angled towards each other (e.g., on the faces of a pyramid). This guarantees energy inflow even after a suboptimal landing.

## 5. Design Requirements
1.  **Form Factor:** Low center of gravity for correct landing.
2.  **Panel Coating:** ETFE (resistance to UV and scratches).
3.  **Thermal Insulation:** The battery compartment should be protected from sharp direct temperature fluctuations.
4.  **Watchdog:** Hardware reset on freeze to prevent deep discharge.

## 6. Conclusion
To create a reliable droppable repeater, it is necessary to use the **RAK4631 + 5000 mAh Li-ion + 3W Solar Panel** combination. This configuration provides a multi-year maintenance-free operational cycle.
