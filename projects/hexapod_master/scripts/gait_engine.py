import math
import numpy as np

class GaitEngine:
    """
    Gait generation engine for hexapod robots.
    Supports Tripod, Wave, and Ripple gaits.
    """
    # Gait types
    TRIPOD = 'tripod'
    WAVE = 'wave'
    RIPPLE = 'ripple'

    def __init__(self, step_height=0.03, step_length=0.05, gait_type='tripod'):
        self.step_height = step_height
        self.step_length = step_length
        self.gait_type = gait_type
        self.phase = 0.0 # 0.0 to 1.0 (Full cycle)
        
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
            }
        }
        self.leg_names = ['rf', 'rm', 'rb', 'lf', 'lm', 'lb']

    def set_gait_type(self, gait_type):
        if gait_type in self.gait_configs:
            self.gait_type = gait_type

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
        # Cycle speed depends on the magnitude of the target velocity
        v_norm = math.sqrt(velocity[0]**2 + velocity[1]**2 + (omega*0.1)**2)
        if v_norm > 0.001:
            self.phase += v_norm * dt * 2.0 # Gait speed coefficient
        if self.phase > 1.0:
            self.phase -= 1.0

        offsets = {}
        
        # 2. Calculate offsets for each leg
        for leg_name in self.leg_names:
            phase_shift = config['offsets'][leg_name]
            leg_phase = (self.phase + phase_shift) % 1.0
            
            # Swing phase
            if leg_phase < swing_dur:
                swing_p = leg_phase / swing_dur # 0.0 to 1.0 within swing
                # X, Y movement (from -Length/2 to +Length/2)
                dx = velocity[0] * self.step_length * (swing_p - 0.5)
                dy = velocity[1] * self.step_length * (swing_p - 0.5)
                # Z lift (parabola)
                dz = self.step_height * math.sin(swing_p * math.pi)
            # Stance phase
            else:
                stance_p = (leg_phase - swing_dur) / stance_dur # 0.0 to 1.0 within stance
                # Move in opposite direction to push the body
                dx = -velocity[0] * self.step_length * (stance_p - 0.5)
                dy = -velocity[1] * self.step_length * (stance_p - 0.5)
                dz = 0.0
                
            offsets[leg_name] = {'x': dx, 'y': dy, 'z': dz}

        return offsets
