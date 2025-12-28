import time

class Interpolator:
    def __init__(self, start_val, speed=0.1):
        """
        start_val: начальное значение (может быть числом, списком или кортежем)
        speed: скорость изменения единиц в секунду
        """
        self.current_val = np.array(start_val, dtype=float)
        self.target_val = np.array(start_val, dtype=float)
        self.speed = speed
        self.last_time = time.time()

    def set_target(self, target):
        self.target_val = np.array(target, dtype=float)

    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Вектор разницы
        diff = self.target_val - self.current_val
        dist = np.linalg.norm(diff)

        if dist < 0.0001:
            self.current_val = self.target_val
            return self.current_val

        # Максимальный шаг за это время
        step = self.speed * dt
        
        if step >= dist:
            self.current_val = self.target_val
        else:
            self.current_val += (diff / dist) * step

        return self.current_val

import numpy as np # Нужен для векторных операций
