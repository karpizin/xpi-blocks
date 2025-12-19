# EPEver Tracer MPPT Solar Controller

The **EPEver Tracer** (A/AN/BN series) is a popular MPPT solar charge controller.
It exposes all data via an **RS-485** port (Modbus RTU protocol).

**Key Metrics:**
*   PV Voltage / Current / Power.
*   Battery Voltage / Current / SoC %.
*   Load Status / Current.
*   Device Temperature.

## ⚡ Wiring (RJ45 to RS-485)

The controller uses a standard RJ45 jack (Ethernet cable), but it is NOT Ethernet.
You need to splice an Ethernet cable or buy a specific "EPEver CC-USB-RS485" cable.

**RJ45 Pinout:**
*   **Pin 1 & 2:** B- (RS485 B)
*   **Pin 3 & 4:** A+ (RS485 A)
*   **Pin 5 & 6:** GND
*   **Pin 7 & 8:** VCC (5V) - *Do not connect VCC to USB adapter unless it needs power.*

| EPEver (RJ45) | USB-RS485 Adapter |
| :--- | :--- |
| **Blue/Blue-White (1,2)** | **B-** |
| **Green/Green-White (3,4)** | **A+** |
| **Orange/Orange-White (5,6)** | **GND** |

## 🚀 Usage

This block uses the `modbus_rtu_node` with a pre-configured register map.

### 1. Launch
```bash
ros2 launch xpi_sensors epever.launch.py
```

### 2. Verify Data
```bash
ros2 topic echo /epever/status
```
*Output: JSON string with keys like `pv_volt`, `batt_soc`, `load_amps`.*

## ⚙️ Logic
It polls the **Input Registers (3xxx)** for live data.
*   **Slave ID:** Default is **1**.
*   **Baudrate:** Default is **115200** (Check your manual, older models use 9600).

## 🧩 Topics Interface

### Publishers
*   `~/status` (`std_msgs/msg/String`) - JSON data.
