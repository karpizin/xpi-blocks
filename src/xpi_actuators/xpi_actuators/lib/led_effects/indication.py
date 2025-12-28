import time
import math

class IndicationEffectsMixin:
    def effect_double_blink(self, color=(255, 100, 0), speed=1.0):
        # speed: cycle frequency
        t = (time.time() * speed) % 2.0
        if 0.0 < t < 0.2 or 0.4 < t < 0.6:
            self.fill(color)
        else:
            self.clear()

    def effect_sos(self, color=(255, 0, 0), speed=1.0):
        # S: . . . (3 short)
        # O: - - - (3 long)
        # S: . . . (3 short)
        # Simple implementation using time phases
        t = (time.time() * speed) % 6.0
        self.clear()
        # S1
        if (0.0 < t < 0.3) or (0.6 < t < 0.9) or (1.2 < t < 1.5): self.fill(color)
        # O
        elif (2.0 < t < 2.8) or (3.1 < t < 3.9) or (4.2 < t < 5.0): self.fill(color)
        # S2
        elif (5.5 < t < 5.8) or (6.1 < t < 6.4) or (6.7 < t < 7.0): self.fill(color)

    def effect_fast_blink(self, color=(255, 255, 255), speed=5.0):
        if (time.time() * speed) % 1 < 0.5:
            self.fill(color)
        else:
            self.clear()
            
    def effect_status_pulse(self, color=(0, 255, 0), speed=1.0):
        # Short blip every second
        if (time.time() * speed) % 1.0 < 0.1:
            self.fill(color)
        else:
            self.clear()
