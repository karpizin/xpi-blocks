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

    def calculate_offsets(self, velocity, omega, dt):
        """
        Calculates leg offsets based on desired velocity and rotation.
        velocity: (vx, vy) - target linear velocity
        omega: angular velocity (yaw)
        dt: time elapsed since last update
        """
        config = self.gait_configs.get(self.gait_type, self.gait_configs[self.TRIPOD])
        swing_dur = config['swing_duration']
        stance_dur = 1.0 - swing_dur

        # 1. Update global cycle phase
        # Cycle speed depends on combined linear and angular velocity
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
            
            # Get leg attachment point for rotation calculations
            l_cfg = self.leg_configs.get(f'leg_{leg_name}', {'x': 0.1, 'y': 0.1})
            lx, ly = l_cfg['x'], l_cfg['y']
            
            # Combine linear velocity and rotational velocity (tangent)
            # v_total = v_linear + (omega x r_leg)
            vx = velocity[0] - omega * ly
            vy = velocity[1] + omega * lx
            
            # Swing phase: lift leg and move to target landing spot
            if leg_phase < swing_dur:
                p = leg_phase / swing_dur # 0.0 to 1.0 within swing
                # Move from -Step/2 to +Step/2
                dx = vx * self.step_length * (p - 0.5)
                dy = vy * self.step_length * (p - 0.5)
                # Parabolic lift
                dz = self.step_height * math.sin(p * math.pi)
            # Stance phase: move leg back to push body forward
            else:
                p = (leg_phase - swing_dur) / stance_dur # 0.0 to 1.0 within stance
                # Move from +Step/2 to -Step/2
                dx = -vx * self.step_length * (p - 0.5)
                dy = -vy * self.step_length * (p - 0.5)
                # Apply terrain adaptation during stance
                dz = self.terrain_offsets.get(leg_name, 0.0)
                
            offsets[leg_name] = {'x': dx, 'y': dy, 'z': dz}

        return offsets
