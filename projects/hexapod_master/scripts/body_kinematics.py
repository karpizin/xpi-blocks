import math
import numpy as np

class BodyKinematics:
    def __init__(self, config):
        """
        config: dict containing 'legs', 'dimensions', and 'default_height'
        """
        self.leg_origins = config['legs']
        self.dims = config['dimensions']
        self.default_height = config.get('default_height', 0.08)
        # Neutral reach calculation: l1 + some fraction of l2/l3 for stability
        self.neutral_reach = self.dims['l1'] + self.dims['l2'] * 0.8 

    def calculate_body_ik(self, translation, rotation):
        """
        Calculates leg tip coordinates relative to their bases (Coxa).
        translation: (x, y, z)
        rotation: (roll, pitch, yaw) in radians
        """
        tx, ty, tz = translation
        roll, pitch, yaw = rotation

        # Rotation matrices (R_x, R_y, R_z unchanged...)
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
            # 1. Leg mount position in body coordinate system
            p_origin = np.array([origin['x'], origin['y'], origin['z']])
            
            # 2. Foot position in the world (Neutral stance)
            p_foot_world = p_origin + np.array([math.cos(origin['angle']) * self.neutral_reach, 
                                               math.sin(origin['angle']) * self.neutral_reach, 
                                               -self.default_height])

            # 3. New mount position after body movement
            p_origin_new = R_body @ p_origin + T_body
            
            # 4. Vector from new mount to foot (in world coordinate system relative to mount)
            foot_rel = p_foot_world - p_origin_new
            
            # 5. Rotate this vector into the leg's local coordinate system
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
