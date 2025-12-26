import time
import math
import random

def wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return (pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return (0, pos * 3, 255 - pos * 3)

def hsv_to_rgb(h, s, v):
    """Convert HSV (0-1, 0-1, 0-1) to RGB (0-255, 0-255, 0-255)."""
    if s == 0.0: return (int(v*255), int(v*255), int(v*255))
    i = int(h*6.0)
    f = (h*6.0) - i
    p, q, t = v*(1.0 - s), v*(1.0 - s*f), v*(1.0 - s*(1.0-f))
    p, q, t = int(p*255), int(q*255), int(t*255)
    v = int(v*255)
    i %= 6
    if i == 0: return (v, t, p)
    if i == 1: return (q, v, p)
    if i == 2: return (p, v, t)
    if i == 3: return (p, q, v)
    if i == 4: return (t, p, v)
    return (v, p, q)

class EffectBase:
    def __init__(self, num_pixels):
        self.num_pixels = num_pixels
        self.pixels = [(0, 0, 0)] * num_pixels
        self.step = 0
        self.last_update = time.time()
        
        # State variables for effects
        self.pos = 0
        self.direction = 1
        self.color_idx = 0
        self.fade_val = 0
        self.balls = [] 

    def clear(self):
        self.pixels = [(0, 0, 0)] * self.num_pixels

    def set_pixel(self, i, color):
        if 0 <= i < self.num_pixels:
            self.pixels[i] = color

    def fill(self, color):
        self.pixels = [color] * self.num_pixels

    def fade_to_black(self, scale=10):
        """Dim all pixels by a scaling factor."""
        for i in range(self.num_pixels):
            r, g, b = self.pixels[i]
            r = 0 if r <= 10 else int(r * (255 - scale) / 255)
            g = 0 if g <= 10 else int(g * (255 - scale) / 255)
            b = 0 if b <= 10 else int(b * (255 - scale) / 255)
            self.pixels[i] = (r, g, b)

    def reset_state(self):
        """Call this when changing effects to clear internal counters."""
        self.step = 0
        self.last_update = time.time()
        self.clear()
