import random
import math
import time
from .base import wheel

class PhysicsEffectsMixin:
    def effect_fire(self, speed=1.0):
        # Already exists in old lib, let's reimplement it cleaner
        # Cooling
        for i in range(self.num_pixels):
            cooldown = random.randint(0, int(((50 * speed) / self.num_pixels) + 2))
            r, g, b = self.pixels[i]
            self.pixels[i] = (max(0, r - cooldown), max(0, g - cooldown), 0)
            
        # Drifting
        for i in range(self.num_pixels - 1, 2, -1):
             self.pixels[i] = self.pixels[i - 1]
             
        # Igniting
        if random.random() < (0.2 * speed):
             y = random.randint(0, min(7, self.num_pixels - 1))
             self.pixels[y] = (255, random.randint(100, 200), 0)

    def effect_blue_fire(self, speed=1.0):
        for i in range(self.num_pixels):
            cooldown = random.randint(0, int(((50 * speed) / self.num_pixels) + 2))
            r, g, b = self.pixels[i]
            self.pixels[i] = (0, max(0, g - cooldown), max(0, b - cooldown))
        for i in range(self.num_pixels - 1, 2, -1):
             self.pixels[i] = self.pixels[i - 1]
        if random.random() < (0.2 * speed):
             y = random.randint(0, min(7, self.num_pixels - 1))
             self.pixels[y] = (0, random.randint(100, 200), 255)

    def effect_water(self, speed=1.0):
        # speed: flow speed
        t = time.time() * speed
        for i in range(self.num_pixels):
            # Overlapping sine waves
            w = math.sin(i * 0.3 + t) * 0.5 + 0.5
            w2 = math.sin(i * 0.1 - t * 0.5) * 0.5 + 0.5
            val = (w + w2) / 2.0
            self.set_pixel(i, (0, int(100 * val), int(255 * val)))

    def effect_plasma(self, speed=1.0):
        t = time.time() * speed
        for i in range(self.num_pixels):
            v1 = math.sin(i * 0.1 + t)
            v2 = math.sin(0.1 * (i * math.sin(t/2.0) + 10 * math.sin(t/3.0)))
            v = (v1 + v2 + 2.0) / 4.0 # Normalize 0-1
            self.set_pixel(i, wheel(int(v * 255)))

    def effect_bubble(self, color=(0, 150, 255), speed=1.0):
        # Decay/Rise
        self.fade_to_black(30)
        for i in range(self.num_pixels - 1, 0, -1):
            if self.pixels[i-1] != (0,0,0) and random.random() < 0.8:
                self.pixels[i] = self.pixels[i-1]
                self.pixels[i-1] = (0,0,0)
        
        # New bubble at bottom
        if random.random() < (0.1 * speed):
            self.set_pixel(0, color)
