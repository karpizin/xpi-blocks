import numpy as np

class Interpolator:
    def __init__(self, start_val, speed=1.0):
        """
        start_val: initial value (can be a number, list, or tuple)
        speed: rate of change in units per second
        """
        self.current = np.array(start_val, dtype=float)
        self.target = np.array(start_val, dtype=float)
        self.speed = speed

    def set_target(self, target):
        self.target = np.array(target, dtype=float)

    def update(self, dt):
        if np.array_equal(self.current, self.target):
            return self.current

        # Difference vector
        diff = self.target - self.current
        dist = np.linalg.norm(diff)

        # Maximum step for this time
        step = self.speed * dt

        if dist <= step:
            self.current = np.copy(self.target)
        else:
            self.current += (diff / dist) * step

        return self.current