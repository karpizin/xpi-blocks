# Project Hexapod Master: Advanced Motion & ML

## 🎯 Project Overview
The goal of this project is to create a high-level control system for a hexapod with a manipulator mounted on it. The project combines classical robotics (inverse kinematics) and modern machine learning methods to achieve maximum smoothness and adaptability of movements.

## 🚀 Key Goals
1.  **Modular Kinematics:** Development of universal IK blocks for 3-DOF limbs and 4-6 DOF manipulators.
2.  **Organic Gait:** Using ML to smooth transitions between steps and dynamic body balancing.
3.  **AI Integration (VLM/LLM):** Controlling robot actions via high-level commands (e.g., "go down the stairs and bring the red cube").
4.  **Hardware-Agnostic Design:** Support for various controllers (PCA9685, Dynamixel) via standard ROS2 interfaces.

## 📂 Structure
*   `docs/`: Technical specifications, calculations, and training strategies.
*   `urdf/`: Robot models for Rviz2 visualization and simulation training.
*   `config/`: Limb geometry parameters, gait settings, and neural network weights.
*   `nodes/` (in development): Specific hexapod control nodes.
