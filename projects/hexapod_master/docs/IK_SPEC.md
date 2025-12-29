# Technical Specification: Leg IK (3-DOF)

## Joint Configuration
1.  **Coxa (Hip):** Rotation around Z-axis (Yaw).
2.  **Femur (Shoulder):** Rotation around Y-axis (Pitch).
3.  **Tibia (Knee):** Rotation around Y-axis (Pitch).

## Mathematical Approach
We use an analytical solution:
1.  Projection onto the XY plane to find the Coxa angle.
2.  Solving the Femur-Tibia triangle in the vertical plane using the Law of Cosines.

## Constraints
*   Tibia collision with the body.
*   Servo angle limits (typically 0-180°).