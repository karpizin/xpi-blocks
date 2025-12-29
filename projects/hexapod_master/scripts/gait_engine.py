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
        
        # State per leg
        self.leg_names = ['rf', 'rm', 'rb', 'lf', 'lm', 'lb']
        self.contacts = {name: False for name in self.leg_names}
        self.contact_heights = {name: 0.0 for name in self.leg_names}
        self.last_leg_phases = {name: 0.0 for name in self.leg_names}
        
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

    def set_gait_type(self, gait_type):
        if gait_type in self.gait_configs:
            self.gait_type = gait_type

    def register_contact(self, leg_name, current_z):
        """Called when a leg contact sensor triggers."""
        if leg_name in self.contacts:
            # We only lock contact if the leg is in the second half of swing (descending)
            # or if it's already in stance.
            self.contacts[leg_name] = True
            self.contact_heights[leg_name] = current_z

    def calculate_offsets(self, velocity, omega, dt):
        """
        Calculates leg offsets with ground adaptation.
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
            
            # Detect new swing cycle to reset contact
            if leg_phase < self.last_leg_phases[leg_name]:
                self.contacts[leg_name] = False
                self.contact_heights[leg_name] = 0.0
            self.last_leg_phases[leg_name] = leg_phase

            l_cfg = self.leg_configs.get(f'leg_{leg_name}', {'x': 0.1, 'y': 0.1})
            lx, ly = l_cfg['x'], l_cfg['y']
            
            vx = velocity[0] - omega * ly
            vy = velocity[1] + omega * lx
            
            stride_x = vx * self.step_length
            stride_y = vy * self.step_length

            # Swing phase
            if leg_phase < swing_dur:
                p = leg_phase / swing_dur
                
                # If we hit ground early (descending), lock the height
                if p > 0.5 and self.contacts[leg_name]:
                    dz = self.contact_heights[leg_name]
                    # Keep X,Y moving to target landing to avoid "jerk"
                    dx = vx * self.step_length * (p - 0.5)
                    dy = vy * self.step_length * (p - 0.5)
                else:
                    p0 = [-stride_x/2, -stride_y/2, 0.0]
                    p1 = [-stride_x/2, -stride_y/2, self.step_height * 1.5]
                    p2 = [stride_x/2, stride_y/2, self.step_height * 1.5]
                    p3 = [stride_x/2, stride_y/2, 0.0]
                    
                    bezier_point = self.calculate_bezier_point(p, p0, p1, p2, p3)
                    dx, dy, dz = bezier_point
                
            # Stance phase
            else:
                p = (leg_phase - swing_dur) / stance_dur
                dx = stride_x/2 - p * stride_x
                dy = stride_y/2 - p * stride_y
                # If leg hasn't hit ground yet, keep moving down (Search Mode)
                if not self.contacts[leg_name]:
                    dz = -0.02 * p # Gently probe deeper (2cm max search)
                else:
                    dz = self.contact_heights[leg_name]
                
            offsets[leg_name] = {'x': dx, 'y': dy, 'z': dz}

        return offsets
