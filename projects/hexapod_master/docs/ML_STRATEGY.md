# Machine Learning Strategy

## 1. Data Collection (Imitation Learning)
Recording telemetry while controlling the robot using classical algorithms:
*   **Input:** Desired velocity vector + IMU data.
*   **Output:** Servo angles.

## 2. Reinforcement Learning
Main Goal: Balancing and minimizing shaking.
*   **Reward Function:** + for velocity matching the target vector, - for IMU deviation from horizontal, - for sharp spikes in servo current/voltage.

## 3. Inference
Using **ONNX Runtime** or **TensorFlow Lite** to execute the model on the Raspberry Pi CPU at a frequency of at least 50 Hz.