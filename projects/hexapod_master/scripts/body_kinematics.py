import math
import numpy as np

class BodyKinematics:
    def __init__(self, leg_origins):
        """
        leg_origins: dict { 'leg_name': {'x':, 'y':, 'z':, 'angle':} }
        """
        self.leg_origins = leg_origins

    def calculate_body_ik(self, translation, rotation):
        """
        Вычисляет координаты кончиков лап относительно их оснований (Coxa).
        translation: (x, y, z)
        rotation: (roll, pitch, yaw) в радианах
        """
        tx, ty, tz = translation
        roll, pitch, yaw = rotation

        # Матрицы вращения
        R_x = np.array([[1, 0, 0],
                        [0, math.cos(roll), -math.sin(roll)],
                        [0, math.sin(roll), math.cos(roll)]])
        
        R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                        [0, 1, 0],
                        [-math.sin(pitch), 0, math.cos(pitch)]])
        
        R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                        [math.sin(yaw), math.cos(yaw), 0],
                        [0, 0, 1]])
        
        R_body = R_z @ R_y @ R_x
        T_body = np.array([tx, ty, tz])

        leg_results = {}

        for name, origin in self.leg_origins.items():
            # 1. Позиция крепления лапы в системе координат корпуса
            p_origin = np.array([origin['x'], origin['y'], origin['z']])
            
            # 2. Позиция "ступни" в мире (предположим, робот стоит на месте)
            # В нейтральном положении ступня находится под креплением на высоте -H
            # Но для Body IK мы считаем смещение относительно нейтральной точки ступни
            p_foot_world = p_origin + np.array([math.cos(origin['angle'])*0.1, 
                                               math.sin(origin['angle'])*0.1, 
                                               -0.08]) # 0.1m reach, 0.08m height

            # 3. Новая позиция крепления лапы после движения корпуса
            p_origin_new = R_body @ p_origin + T_body
            
            # 4. Вектор от нового крепления к ступне (в системе координат корпуса)
            # Это и есть цель для Leg IK
            foot_rel = p_foot_world - p_origin_new
            
            # 5. Поворачиваем этот вектор в локальную систему координат лапы
            # Чтобы ось X лапы всегда смотрела "от корпуса"
            inv_angle = -origin['angle']
            R_leg = np.array([[math.cos(inv_angle), -math.sin(inv_angle), 0],
                             [math.sin(inv_angle), math.cos(inv_angle), 0],
                             [0, 0, 1]])
            
            foot_local = R_leg @ foot_rel
            
            leg_results[name] = {
                'x': foot_local[0],
                'y': foot_local[1],
                'z': foot_local[2]
            }

        return leg_results
