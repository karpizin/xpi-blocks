# Inverse Kinematics Mathematics (3-DOF Leg)

This document describes the process of translating target coordinates $(x, y, z)$ into rotation angles for the three servos of a hexapod leg.

## Leg Scheme and Parameters
The leg consists of three links:
1.  **Coxa (L1)**: Hip (horizontal rotation).
2.  **Femur (L2)**: Upper leg (vertical rotation).
3.  **Tibia (L3)**: Lower leg (vertical rotation).

**Input Data:**
*   $x, y, z$ — coordinates of the tiptoe relative to the Coxa rotation center.
*   $L_1, L_2, L_3$ — lengths of the respective links.

---

## 1. Coxa Angle Calculation ($ \theta_1 $)
This is the rotation in the XY plane.
$$ \theta_1 = \text{atan2}(y, x) $$

## 2. Transition to 2D Leg Plane
To find the Femur and Tibia angles, we project the leg onto the vertical plane defined by the angle $ \theta_1 $.
*   Horizontal distance from the Femur joint to the target:
    $$ r = \sqrt{x^2 + y^2} - L_1 $$
*   Full distance from the Femur joint to the target (hypotenuse):
    $$ s = \sqrt{r^2 + z^2} $$

## 3. Femur ($ \theta_2 $) and Tibia ($ \theta_3 $) Angles Calculation
We use the Law of Cosines for the triangle formed by links $ L_2, L_3 $, and the imaginary segment $ s $.

### Tibia Angle ($ \theta_3 $)
$$ \cos( \theta_3) = \frac{L_2^2 + L_3^2 - s^2}{2 L_2 L_3} $$
$$ \theta_3 = \text{acos}(\cos( \theta_3)) - \pi $$
*(Sign depends on the servo rotation direction)*

### Femur Angle ($ \theta_2 $)
First, find the inclination angle of segment $ s $ relative to the horizon:
$$ \alpha = \text{atan2}(z, r) $$
Then, find the internal angle of the triangle at the Femur joint:
$$ \cos(\beta) = \frac{L_2^2 + s^2 - L_3^2}{2 L_2 s} $$
$$ \beta = \text{acos}(\cos(\beta)) $$
Final angle:
$$ \theta_2 = \alpha + \beta $$

---

## Constraints and Checks
1.  **Reachability Check:** If $ s > L_2 + L_3 $, the point is out of reach.
2.  **Singularity:** If $ s < |L_2 - L_3| $, the leg folds into itself.
3.  **Servo Angles:** Resulting radians must be converted to degrees and clamped to the servo range (usually 0-180°).