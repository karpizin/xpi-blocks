import math

class LegKinematics:
    def __init__(self, l1, l2, l3):
        """
        Инициализация длин звеньев лапы.
        l1: Coxa
        l2: Femur
        l3: Tibia
        """
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

    def calculate_ik(self, x, y, z):
        """
        Расчет обратной кинематики.
        Возвращает кортеж (coxa_angle, femur_angle, tibia_angle) в радианах.
        """
        # 1. Угол Coxa
        theta1 = math.atan2(y, x)

        # 2. Расстояния в 2D плоскости лапы
        r = math.sqrt(x**2 + y**2) - self.l1
        s = math.sqrt(r**2 + z**2)

        # Проверка достижимости
        if s > (self.l2 + self.l3):
            raise ValueError(f"Точка ({x}, {y}, {z}) вне зоны досягаемости (s={s:.2f})")
        if s < abs(self.l2 - self.l3):
            raise ValueError(f"Точка ({x}, {y}, {z}) слишком близко (s={s:.2f})")

        # 3. Угол Tibia (по теореме косинусов)
        cos_gamma = (self.l2**2 + self.l3**2 - s**2) / (2 * self.l2 * self.l3)
        # Ограничиваем точность для acos
        cos_gamma = max(-1.0, min(1.0, cos_gamma))
        theta3 = math.acos(cos_gamma) - math.pi

        # 4. Угол Femur
        alpha1 = math.atan2(z, r)
        cos_beta = (self.l2**2 + s**2 - self.l3**2) / (2 * self.l2 * s)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        alpha2 = math.acos(cos_beta)
        theta2 = alpha1 + alpha2

        return theta1, theta2, theta3

    def to_degrees(self, angles):
        return tuple(math.degrees(a) for a in angles)

# Простой тест
if __name__ == "__main__":
    # Пример параметров: Coxa=30mm, Femur=50mm, Tibia=80mm
    leg = LegKinematics(30, 50, 80)
    
    try:
        target = (80, 0, -40)
        angles_rad = leg.calculate_ik(*target)
        angles_deg = leg.to_degrees(angles_rad)
        
        print(f"Target: {target}")
        print(f"Angles (Rad): Coxa={angles_rad[0]:.2f}, Femur={angles_rad[1]:.2f}, Tibia={angles_rad[2]:.2f}")
        print(f"Angles (Deg): Coxa={angles_deg[0]:.2f}, Femur={angles_deg[1]:.2f}, Tibia={angles_deg[2]:.2f}")
    except ValueError as e:
        print(f"Error: {e}")
