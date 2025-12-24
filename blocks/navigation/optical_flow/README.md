# Navigation: PMW3901 Optical Flow Sensor (SPI)

The PMW3901 is a high-performance optical flow sensor that tracks the movement of the floor relative to the robot. It is the gold standard for indoor navigation where GPS is unavailable and wheel encoders might slip.

## 📌 Features
*   **Tracking Range:** 80mm to infinity (optimal up to 2-3 meters).
*   **Update Rate:** Up to 100Hz.
*   **Interface:** SPI (up to 2MHz).
*   **Output:** Delta X and Delta Y in "flow pixels".

## 🔌 Wiring Diagram

| PMW3901 Pin | Raspberry Pi Pin | Color (suggested) | Note | 
| :--- | :--- | :--- | :--- | 
| VIN | 3.3V (Pin 1) | Red | | 
| GND | GND (Pin 9) | Black | | 
| SCLK | SCLK (GPIO 11 / Pin 23) | Yellow | SPI Clock | 
| MOSI | MOSI (GPIO 10 / Pin 19) | Blue | SPI Out | 
| MISO | MISO (GPIO 9 / Pin 21) | Green | SPI In | 
| CS | CE0 (GPIO 8 / Pin 24) | White | SPI Chip Select | 

## 📐 How it Works (Metric Output)
The sensor outputs raw pixel displacement ($ \Delta P $). To get real-world distance ($ \Delta D $), you need to know the height ($ H $) above the floor:

$$ \Delta D = \frac{\Delta P \cdot H}{\text{Scalar}} $$

By default, the scalar for PMW3901 is approximately **242**. Our ROS2 node can automatically perform this calculation if you provide a height value or connect a distance sensor (like VL53L1X).

## 🚀 Quick Start

1.  **Enable SPI:** Use `raspi-config` to enable the SPI interface.
2.  **Install Dependencies:**
    ```bash
    pip install pmw3901
    ```
3.  **Run the Node:**
    ```bash
    ros2 launch xpi_navigation optical_flow.launch.py
    ```
4.  **Monitor Velocity:**
    ```bash
    ros2 topic echo /optical_flow/velocity
    ```

## 📊 Published Topics
*   `~/velocity` ([geometry_msgs/TwistStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistStamped.html)): Current X/Y velocity in m/s.
*   `~/displacement` ([geometry_msgs/Point](http://docs.ros.org/en/api/geometry_msgs/html/msg/Point.html)): Total integrated distance since startup (meters).
*   `~/raw_flow` ([std_msgs/Int32MultiArray](http://docs.ros.org/en/api/std_msgs/html/msg/Int32MultiArray.html)): Raw [dx, dy] pixels from the sensor.

## 🛰️ Subscribed Topics (Optional)
*   `/vl53l1x/range` ([sensor_msgs/Range](http://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)): Used for real-time height compensation.
