import random
import math
import time
from .base import hsv_to_rgb

class MeditativeEffectsMixin:
    def effect_starry_night(self, speed=1.0):
        # Deep blue background with slow twinkling stars
        for i in range(self.num_pixels):
            # Base deep blue
            self.set_pixel(i, (0, 0, 20))
            
        # Twisted noise logic for twinkling
        t = time.time() * speed
        for i in range(self.num_pixels):
            # Each pixel has its own twinkling phase
            noise = math.sin(i * 0.5 + t * 0.2) * math.cos(i * 0.8 - t * 0.3)
            if noise > 0.8:
                # Twinkle star
                brightness = int((noise - 0.8) * 5 * 255)
                self.set_pixel(i, (brightness, brightness, 255))

    def effect_firefly_field(self, speed=1.0):
        # Dark green base
        self.fade_to_black(10) # Slow decay
        for i in range(self.num_pixels):
            r, g, b = self.pixels[i]
            if r == 0 and g == 0 and b == 0:
                self.set_pixel(i, (0, 5, 0)) # Very dim green background
        
        # Chance to spawn a firefly
        if random.random() < (0.03 * speed):
            idx = random.randint(0, self.num_pixels - 1)
            # Yellow-green firefly
            self.set_pixel(idx, (150, 255, 0))

    def effect_fireplace(self, speed=1.0):
        # Embers effect
        t = time.time() * speed
        for i in range(self.num_pixels):
            # Base glow
            pulse = math.sin(t + i * 0.2) * 0.2 + 0.8
            # Per-pixel flicker
            flicker = random.uniform(0.7, 1.0)
            
            r = int(255 * pulse * flicker)
            g = int(random.randint(20, 60) * pulse * flicker)
            self.set_pixel(i, (r, g, 0))

    def effect_calm_ocean(self, speed=0.5):
        # Slow morphing of blues and cyans
        t = time.time() * speed
        for i in range(self.num_pixels):
            # Use HSV for smoother transitions
            hue = 0.55 + math.sin(i * 0.1 + t * 0.1) * 0.05 # Blue range
            sat = 0.8 + math.cos(i * 0.05 - t * 0.05) * 0.2
            val = 0.4 + math.sin(i * 0.2 + t * 0.2) * 0.3
            self.set_pixel(i, hsv_to_rgb(hue, sat, val))

    def effect_sunny_forest(self, speed=0.5):
        # Greens with sunbeams
        t = time.time() * speed
        for i in range(self.num_pixels):
            # Base green
            base_hue = 0.25 + math.sin(i * 0.05) * 0.05 # Green range
            
            # Sunbeams (moving spots of light)
            beam = math.sin(i * 0.1 - t * 0.2) * math.cos(i * 0.05 + t * 0.1)
            if beam > 0.6:
                # Golden sunbeam
                brightness = (beam - 0.6) * 2.5
                rgb = hsv_to_rgb(0.12, 0.6, brightness) # Gold
                self.set_pixel(i, rgb)
            else:
                # Regular leaf shade
                self.set_pixel(i, hsv_to_rgb(base_hue, 0.9, 0.3))
