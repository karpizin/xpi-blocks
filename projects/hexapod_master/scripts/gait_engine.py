import math
import numpy as np

class GaitEngine:
    def __init__(self, step_height=0.03, step_length=0.05):
        self.step_height = step_height
        self.step_length = step_length
        self.phase = 0.0 # 0.0 to 1.0 (Full cycle)
        
        # Группы лап (Tripod)
        self.group_a = ['rf', 'lm', 'rb']
        self.group_b = ['lf', 'rm', 'lb']

    def calculate_offsets(self, velocity, omega, dt):
        """
        velocity: (vx, vy) - желаемая скорость
        omega: угловая скорость (yaw)
        dt: время с последнего обновления
        """
        # 1. Обновляем глобальную фазу цикла
        # Скорость цикла зависит от величины желаемой скорости
        v_norm = math.sqrt(velocity[0]**2 + velocity[1]**2 + (omega*0.1)**2)
        if v_norm > 0.001:
            self.phase += v_norm * dt * 2.0 # Коэффициент скорости походки
        if self.phase > 1.0:
            self.phase -= 1.0

        offsets = {}
        
        # 2. Расчет для каждой группы
        for leg_group, phase_shift in [(self.group_a, 0.0), (self.group_b, 0.5)]:
            # Индивидуальная фаза лапы (с учетом сдвига группы)
            leg_phase = (self.phase + phase_shift) % 1.0
            
            # Фаза переноса (Swing): 0.0 -> 0.5
            if leg_phase < 0.5:
                swing_p = leg_phase * 2.0 # 0.0 to 1.0 within swing
                # X, Y движение (от -Length/2 до +Length/2)
                dx = velocity[0] * self.step_length * (swing_p - 0.5)
                dy = velocity[1] * self.step_length * (swing_p - 0.5)
                # Z подъем (парабола)
                dz = self.step_height * math.sin(swing_p * math.pi)
            # Фаза опоры (Stance): 0.5 -> 1.0
            else:
                stance_p = (leg_phase - 0.5) * 2.0 # 0.0 to 1.0 within stance
                # Движемся в обратном направлении, чтобы толкать корпус
                dx = -velocity[0] * self.step_length * (stance_p - 0.5)
                dy = -velocity[1] * self.step_length * (stance_p - 0.5)
                dz = 0.0

            for leg_name in leg_group:
                offsets[leg_name] = {'x': dx, 'y': dy, 'z': dz}

        return offsets
