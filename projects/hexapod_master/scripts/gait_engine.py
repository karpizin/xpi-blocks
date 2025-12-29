import math
import numpy as np

class GaitEngine:
    """
    Gait generation engine for hexapod robots.
    Supports Tripod, Wave, Ripple, and Amble gaits.
    Handles omnidirectional movement and rotation.
    """
    # Gait types
    TRIPOD = 'tripod'
    WAVE = 'wave'
    RIPPLE = 'ripple'
    AMBLE = 'amble'

    def __init__(self, step_height=0.03, step_length=0.05, gait_type='tripod', leg_configs=None):
        self.step_height = step_height
        self.step_length = step_length
        self.gait_type = gait_type
        self.phase = 0.0 # 0.0 to 1.0 (Full cycle)
        self.leg_configs = leg_configs or {}
        
        # Terrain adaptation offsets (per leg)
        self.terrain_offsets = {name: 0.0 for name in ['rf', 'rm', 'rb', 'lf', 'lm', 'lb']}
        
        # Leg groups/offsets configuration
        self.gait_configs = {
            self.TRIPOD: {
                'swing_duration': 0.5,
                'offsets': {
                    'rf': 0.0, 'lm': 0.0, 'rb': 0.0,
                    'lf': 0.5, 'rm': 0.5, 'lb': 0.5
                }
            },
            self.WAVE: {
                'swing_duration': 1.0/6.0,
                'offsets': {
                    'rf': 0.0, 'rm': 1.0/6.0, 'rb': 2.0/6.0,
                    'lb': 3.0/6.0, 'lm': 4.0/6.0, 'lf': 5.0/6.0
                }
            },
            self.RIPPLE: {
                'swing_duration': 1.0/3.0,
                'offsets': {
                    'rf': 0.0, 'lb': 0.0,
                    'rm': 1.0/3.0, 'lf': 1.0/3.0,
                    'rb': 2.0/3.0, 'lm': 2.0/3.0
                }
            },
            self.AMBLE: {
                'swing_duration': 0.25,
                'offsets': {
                    'rf': 0.0, 'lb': 0.5,
                    'rm': 0.25, 'lf': 0.75,
                    'rb': 0.5, 'lm': 0.0
                }
            }
        }
        self.leg_names = ['rf', 'rm', 'rb', 'lf', 'lm', 'lb']

    def set_gait_type(self, gait_type):
        if gait_type in self.gait_configs:
            self.gait_type = gait_type

    def set_terrain_offsets(self, offsets):
        """Sets Z-axis correction for each leg based on terrain feedback."""
        self.terrain_offsets.update(offsets)

    def calculate_bezier_point(self, t, p0, p1, p2, p3):
        """
        Calculates a point on a cubic Bezier curve.
        t: normalized time [0, 1]
        p0..p3: control points (x, y, z)
        """
        res = [0.0, 0.0, 0.0]
        for i in range(3):
            res[i] = (1-t)**3 * p0[i] + 3*(1-t)**2 * t * p1[i] + 3*(1-t) * t**2 * p2[i] + t**3 * p3[i]
        return res

    def calculate_offsets(self, velocity, omega, dt):
        """
        Calculates leg offsets based on desired velocity and rotation.
        Uses Cubic Bezier for swing phase and linear interpolation for stance.
        """
        config = self.gait_configs.get(self.gait_type, self.gait_configs[self.TRIPOD])
        swing_dur = config['swing_duration']
        stance_dur = 1.0 - swing_dur

        # 1. Update global cycle phase
        v_norm = math.sqrt(velocity[0]**2 + velocity[1]**2 + (omega * 0.2)**2)
        if v_norm > 0.001:
            self.phase += v_norm * dt * 2.0
        if self.phase > 1.0:
            self.phase -= 1.0

        offsets = {}
        
        # 2. Calculate offsets for each leg
        for leg_name in self.leg_names:
            phase_shift = config['offsets'].get(leg_name, 0.0)
            leg_phase = (self.phase + phase_shift) % 1.0
            
            l_cfg = self.leg_configs.get(f'leg_{leg_name}', {'x': 0.1, 'y': 0.1})
            lx, ly = l_cfg['x'], l_cfg['y']
            
            vx = velocity[0] - omega * ly
            vy = velocity[1] + omega * lx
            
            # Target stride length per update
            stride_x = vx * self.step_length
            stride_y = vy * self.step_length

            # Swing phase: Use Bezier for smooth motion
            if leg_phase < swing_dur:
                p = leg_phase / swing_dur # 0.0 -> 1.0
                
                # Define 4 control points for the Bezier curve
                # P0: Start (Ground)
                # P1: Lift-off (Same as P0 but with Z)
                # P2: Approach (Same as P3 but with Z)
                # P3: Target Landing
                p0 = [-stride_x/2, -stride_y/2, 0.0]
                p1 = [-stride_x/2, -stride_y/2, self.step_height * 1.5]
                p2 = [stride_x/2, stride_y/2, self.step_height * 1.5]
                p3 = [stride_x/2, stride_y/2, 0.0]
                
                bezier_point = self.calculate_bezier_point(p, p0, p1, p2, p3)
                dx, dy, dz = bezier_point
                
            # Stance phase: Linear push back
            else:
                p = (leg_phase - swing_dur) / stance_dur # 0.0 -> 1.0
                # Move from +HalfStride to -HalfStride
                dx = stride_x/2 - p * stride_x
                dy = stride_y/2 - p * stride_y
                # Apply terrain adaptation during stance
                dz = self.terrain_offsets.get(leg_name, 0.0)
                
            offsets[leg_name] = {'x': dx, 'y': dy, 'z': dz}

        return offsets
