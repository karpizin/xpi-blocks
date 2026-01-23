import math

class ManipulatorKinematics:
    def __init__(self, config):
        """
        config: dict containing 'arm' dimensions
        """
        self.l1 = config['arm']['l1']
        self.l2 = config['arm']['l2']
        self.l3 = config['arm']['l3']

    def calculate_arm_ik(self, x, y, z):
        """
        Calculates manipulator inverse kinematics.
        Target (x, y, z) relative to arm_base_joint.
        Returns: (base_yaw, shoulder_pitch, elbow_pitch)
        """
        # 1. Base Rotation (Yaw)
        base_yaw = math.atan2(y, x)

        # 2. Horizontal distance from base to target
        r = math.sqrt(x**2 + y**2)
        
        # 3. Vertical distance relative to the first joint (shoulder)
        # Note: we assume shoulder is at height l1
        z_rel = z - self.l1
        
        # Distance from shoulder to target
        s = math.sqrt(r**2 + z_rel**2)

        # Reachability Check
        if s > (self.l2 + self.l3) or s < abs(self.l2 - self.l3):
            raise ValueError(f"Manipulator target ({x}, {y}, {z}) is out of reach.")

        # 4. Elbow Pitch (using Law of Cosines)
        cos_elbow = (self.l2**2 + self.l3**2 - s**2) / (2 * self.l2 * self.l3)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow_pitch = math.pi - math.acos(cos_elbow)

        # 5. Shoulder Pitch
        alpha = math.atan2(z_rel, r)
        cos_beta = (self.l2**2 + s**2 - self.l3**2) / (2 * self.l2 * s)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        shoulder_pitch = alpha + beta

        # Note: Depending on your URDF orientation, you might need to adjust offsets
        return (base_yaw, shoulder_pitch, elbow_pitch)
