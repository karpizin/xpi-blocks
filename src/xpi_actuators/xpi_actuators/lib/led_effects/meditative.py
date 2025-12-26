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

    def effect_aurora_borealis(self, speed=0.3):
        # Slow waving of green, purple, blue
        t = time.time() * speed
        for i in range(self.num_pixels):
            # Complex wave for "dancing" feel
            w1 = math.sin(i * 0.1 + t)
            w2 = math.sin(i * 0.2 - t * 0.5)
            mix = (w1 + w2) / 2.0
            
            hue = 0.4 + mix * 0.3 # Green (0.33) to Purple (0.8)
            sat = 0.7 + mix * 0.3
            val = 0.2 + mix * 0.4
            self.set_pixel(i, hsv_to_rgb(hue, sat, val))

    def effect_zen_pulse(self, color=(255, 150, 50), speed=0.1):
        # Very slow breathing (10s cycle default)
        t = time.time() * speed * 2 * math.pi
        # Use smooth sin wave 0 to 1
        scale = (math.sin(t) + 1.0) / 2.0
        # Make it even smoother at edges
        scale = math.pow(scale, 2)
        r, g, b = color
        self.fill((int(r*scale), int(g*scale), int(b*scale)))

    def effect_morning_mist(self, speed=0.2):
        # Moving clouds of grey-blue and white
        t = time.time() * speed
        for i in range(self.num_pixels):
            w = math.sin(i * 0.05 + t) * math.cos(i * 0.1 - t * 0.3)
            val = (w + 1.0) / 2.0 # 0 to 1
            # Mix between deep blue-grey and bright white
            r = int(50 + 205 * val)
            g = int(70 + 185 * val)
            b = int(100 + 155 * val)
            self.set_pixel(i, (r, g, b))

    def effect_autumn_leaves(self, speed=0.5):
        # Red/Brown background with occasional bright orange drift
        self.fade_to_black(5)
        for i in range(self.num_pixels):
            r, g, b = self.pixels[i]
            if r < 20 and g < 10:
                # Dim background: dark red/brown
                self.set_pixel(i, (20, 5, 0))
        
        # Falling leaf
        if random.random() < (0.02 * speed):
            idx = random.randint(0, self.num_pixels - 1)
            # Bright Gold/Orange
            self.set_pixel(idx, (255, 150, 0))

    def effect_deep_space(self, speed=0.2):
        # Black with occasional nebula gas clouds
        self.fade_to_black(10)
        t = time.time() * speed
        for i in range(self.num_pixels):
            # Large scale noise for nebulas
            noise = math.sin(i * 0.05 + t) * math.cos(i * 0.02 - t * 0.1)
            if noise > 0.7:
                # Nebula color (Cyan or Purple)
                c_idx = (int(t) % 2)
                if c_idx == 0:
                    rgb = (0, int(100 * noise), int(150 * noise)) # Cyan
                else:
                    rgb = (int(100 * noise), 0, int(150 * noise)) # Purple
                
                # Blend with current pixel
                r, g, b = self.pixels[i]
                nr, ng, nb = rgb
                self.set_pixel(i, (max(r, nr), max(g, ng), max(b, nb)))

