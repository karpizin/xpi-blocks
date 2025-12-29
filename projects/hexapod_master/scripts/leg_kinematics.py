import math

class LegKinematics:
    def __init__(self, params):
        """Initializes leg link lengths."""
        self.l1 = params['l1'] # Coxa
        self.l2 = params['l2'] # Femur
        self.l3 = params['l3'] # Tibia

    def calculate_ik(self, x, y, z):
        """
        Calculates inverse kinematics.
        Returns a tuple (coxa_angle, femur_angle, tibia_angle) in radians.
        """
        # 1. Coxa Angle
        coxa_angle = math.atan2(y, x)

        # 2. Distances in 2D leg plane
        r = math.sqrt(x**2 + y**2) - self.l1
        s = math.sqrt(r**2 + z**2)

        # Reachability Check
        if s > (self.l2 + self.l3):
            raise ValueError(f"Point ({x}, {y}, {z}) is out of reach (s={s:.2f})")
        if s < abs(self.l2 - self.l3):
            raise ValueError(f"Point ({x}, {y}, {z}) is too close (s={s:.2f})")

        # 3. Tibia Angle (using Law of Cosines)
        cos_t3 = (self.l2**2 + self.l3**2 - s**2) / (2 * self.l2 * self.l3)
        # Precision clamping for acos
        cos_t3 = max(-1.0, min(1.0, cos_t3))
        tibia_angle = math.acos(cos_t3) - math.pi

        # 4. Femur Angle
        alpha = math.atan2(z, r)
        cos_beta = (self.l2**2 + s**2 - self.l3**2) / (2 * self.l2 * s)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        femur_angle = alpha + beta

        return (coxa_angle, femur_angle, tibia_angle)

# Simple test
if __name__ == "__main__":
    # Example params: Coxa=30mm, Femur=50mm, Tibia=80mm
    leg = LegKinematics({'l1': 30, 'l2': 50, 'l3': 80})
    try:
        angles = leg.calculate_ik(100, 0, -50)
        print(f"Angles: {list(map(math.degrees, angles))}")
    except ValueError as e:
        print(e)