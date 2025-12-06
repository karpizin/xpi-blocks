# XPI-Blocks: LLM/VLM Integration Scenarios

This document outlines potential scenarios for integrating Large Language Models (LLM) and Vision-Language Models (VLM) into the `xpi-blocks` project for intelligent control and sensor data analysis.

## Group A: Analysis of Discrete Sensors and Simple Patterns (Relay, Sonar)

### A.1. Anomaly Detection (Relay State)
*   **Input:** Stream of `std_msgs/Bool` messages from `relay_node` (relay state).
*   **LLM Task:** Monitor if the relay is in an abnormal state (e.g., too long ON/OFF, or changing state too frequently/infrequently).
*   **Output:** Anomaly notification ("Relay 1 has been ON for 5 minutes, exceeding normal operation time.").

### A.2. Trend Identification (Sonar) - **IMPLEMENTED**
*   **Input:** Stream of `sensor_msgs/Range` messages from `sonar_node`.
*   **LLM Task:** Analyze changes in distance over time. Identify trends: "approaching an object", "moving away", "moving parallel to a wall".
*   **Output:** Textual description of the trend ("Moving away from obstacle", "Approaching wall at 0.1 m/s").

### A.3. Pattern Recognition (Simple Sonar) - **IMPLEMENTED**
*   **Input:** Stream of `sensor_msgs/Range`.
*   **LLM Task:** Detect simple patterns: "Object at fixed distance", "Object at distance X, then disappeared, then reappeared".
*   **Output:** Description of the detected pattern ("Object detected at 0.5m, then disappeared for 3 seconds, then reappeared.").

## Group B: Analysis of Continuous Sensors and More Complex Patterns (IMU - to be implemented)

### B.1. Unusual Motion Detection (IMU)
*   **Input:** Stream of `sensor_msgs/Imu` messages (linear acceleration, angular velocity, orientation).
*   **LLM Task:** Detect movements outside "normal" bounds: "Robot fell down", "Strong vibration", "Sudden change in course".
*   **Output:** Event description ("Robot likely fell down, high angular velocity detected around X axis", "Sudden impact detected").

### B.2. Activity Classification (IMU)
*   **Input:** Stream of `sensor_msgs/Imu`.
*   **LLM Task:** Classify robot's state: "Moving straight", "Turning", "Stationary", "Shaking".
*   **Output:** Textual description of the state ("Robot is currently rotating left", "Robot is stationary").

### B.3. Navigation Interpretation (Sonar + IMU)
*   **Input:** Streams of `sensor_msgs/Range` and `sensor_msgs/Imu`.
*   **LLM Task:** Joint analysis to understand environment and motion: "Robot moving forward, wall at 0.3m ahead, needs to turn", "Robot is stuck".
*   **Output:** Generalized navigation status ("Blocked by an object ahead, needs to turn right", "Successfully moved 1 meter forward").

## Group C: High-Level Interpretation and Control (VLM, Tool Calling)

### C.1. Object Recognition (VLM)
*   **Input:** Camera frames (image stream) + textual query ("What do you see?", "Find red object").
*   **VLM Task:** Identify objects in the image.
*   **LLM Task:** Answer user query using VLM data ("I see a blue cup on the table and a book next to it").

### C.2. Intelligent Actuator Control (Tool Calling) - **IMPLEMENTED**
*   **Input:** User text command ("Turn on Relay 1", "Set Servo 0 to 90 degrees", "Start motor").
*   **LLM Task:** Interpret command and invoke corresponding ROS2 service/topic using a "Tool Calling" mechanism for our actuators (Relay, PCA9685).
*   **Output:** Invocation of appropriate ROS2 interface.

### C.3. Contextual Control (VLM + Tool Calling)
*   **Input:** Camera feed, user command ("Pick up the blue object").
*   **VLM Task:** Determine location of the blue object.
*   **LLM Task:** Plan and execute actions: "Drive to the blue object", "Pick it up" (invoking movement and grasping services).
*   **Output:** Sequence of ROS2 interface calls.

## Group D: Long-term Analysis and Proactive Actions

### D.1. Proactive Warning (Multiple Sensors)
*   **Input:** Data streams from multiple sensors (e.g., temperature, humidity, gas, battery charge).
*   **LLM Task:** Identify potential issues before they become critical: "Battery charge low, needs recharging", "Motor temperature exceeding norm".
*   **Output:** Warning to user ("Battery level critically low, seeking charging station.", "Motor temperature is high, reducing speed.").

### D.2. Task Planning with Feedback (LLM + VLM + Actuators)
*   **Input:** High-level goal ("Clear the table"), sensor/VLM streams.
*   **LLM Task:** Create a step-by-step plan, execute it using Tool Calling, and adapt based on feedback from VLM/sensors.
*   **Output:** Execution of complex task with adaptation.

### D.3. Learning and Adaptation (via LLM)
*   **Input:** History of robot actions and states, human feedback.
*   **LLM Task:** Analyze successful/unsuccessful scenarios, suggest changes in behavior or settings.
*   **Output:** Recommendations for optimization or new behavior strategies.
