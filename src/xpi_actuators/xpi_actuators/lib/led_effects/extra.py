import random
import time
import math
from .base import hsv_to_rgb

class ExtraEffectsMixin:
    def effect_police(self, speed=5.0):
        # Red and Blue strobe
        # Pattern: R R R - B B B - R R R - B B B
        t = int(time.time() * 10 * speed)
        phase = t % 4
        
        self.clear()
        if phase == 0:
            for i in range(0, self.num_pixels // 2):
                self.set_pixel(i, (255, 0, 0))
        elif phase == 2:
            for i in range(self.num_pixels // 2, self.num_pixels):
                self.set_pixel(i, (0, 0, 255))
        # Phases 1 and 3 are black (pause)

    def effect_fire_2012(self, cooling=55, sparking=120, speed=15.0):
        # Classic Fire 2012 algorithm tailored for 1D strip
        # Requires persistent state for heat
        if not hasattr(self, '_heat'):
            self._heat = [0] * self.num_pixels

        # 1. Cool down every cell a little
        for i in range(self.num_pixels):
            cooldown = random.randint(0, int(((cooling * 10) / self.num_pixels) + 2))
            self._heat[i] = max(0, self._heat[i] - cooldown)

        # 2. Heat from each cell drifts 'up' and diffuses a little
        for i in range(self.num_pixels - 1, 2, -1):
            self._heat[i] = (self._heat[i - 1] + self._heat[i - 2] + self._heat[i - 2]) // 3

        # 3. Randomly ignite new 'sparks' near the bottom
        if random.randint(0, 255) < sparking:
            y = random.randint(0, 7)
            if y < self.num_pixels:
                self._heat[y] = min(255, self._heat[y] + random.randint(160, 255))

        # 4. Convert heat to color
        for i in range(self.num_pixels):
            self.set_pixel(i, self._heat_to_color(self._heat[i]))

    def _heat_to_color(self, temperature):
        # Heat is 0-255
        # 0 -> Black
        # 255 -> White
        # Scale 'Heat' down from 0-255 to 0-191
        t192 = int((temperature / 255.0) * 191)
        
        # Calculate ramp up from
        heatramp = t192 & 0x3F # 0..63
        heatramp <<= 2 # scale up to 0..252
        
        if t192 > 0x80: # Hottest
            return (255, 255, heatramp)
        elif t192 > 0x40: # Middle
            return (255, heatramp, 0)
        else: # Coolest
            return (heatramp, 0, 0)

    def effect_candy_cane(self, speed=2.0):
        # Red and White stripes moving
        self.step += speed
        offset = int(self.step)
        for i in range(self.num_pixels):
            if (i + offset) % 10 < 5:
                self.set_pixel(i, (255, 0, 0)) # Red
            else:
                self.set_pixel(i, (200, 200, 200)) # White (dimmed)

    def effect_confetti(self, speed=1.0):
        # Random colored speckles that blink in and fade smoothly
        self.fade_to_black(10)
        if random.random() < (0.05 * speed):
            idx = random.randint(0, self.num_pixels - 1)
            # Random HSV color
            rgb = hsv_to_rgb(random.random(), 1.0, 1.0)
            self.set_pixel(idx, rgb)

    def effect_juggle(self, speed=1.0):
        # Eight colored dots, weaving in and out of sync with each other
        self.fade_to_black(20)
        curr_time = time.time() * speed
        for i in range(8):
            # i/8.0 moves the wave phase
            pos = int((self.num_pixels - 1) * ((math.sin(curr_time + i/2.0) + 1.0) / 2.0))
            # Color cycling
            rgb = hsv_to_rgb((curr_time * 0.1 + i/8.0) % 1.0, 1.0, 1.0)
            
            # Blend with existing
            # Simple overwrite or add? Add is better for crossing
            r, g, b = self.pixels[pos]
            nr, ng, nb = rgb
            self.set_pixel(pos, (min(255, r+nr), min(255, g+ng), min(255, b+nb)))

    def effect_fire_blue(self, cooling=55, sparking=120, speed=15.0):
        """Blue (Gas) Fire"""
        self.effect_fire_2012(cooling, sparking, speed)
        # Remap colors from Red/Yellow to Blue/Cyan
        # Fire 2012 produces (Heat, Heat, 0) mostly.
        # We want (0, Heat, Heat) or similar.
        for i in range(self.num_pixels):
            r, g, b = self.pixels[i]
            # Swap channels: R->B, G->G, B->R (but B is usually 0)
            # Standard fire: R=High, G=Med, B=Low
            # Blue fire: R=Low, G=Med, B=High
            self.set_pixel(i, (b, g, r))

    def effect_fire_plasma(self, cooling=50, sparking=120, speed=15.0):
        """Green/Purple Plasma Fire"""
        self.effect_fire_2012(cooling, sparking, speed)
        for i in range(self.num_pixels):
            r, g, b = self.pixels[i]
            # Map heat to Green/Purple
            # R (Heat) -> G
            # G (Heat/2) -> B
            # B -> R (Purple tint)
            self.set_pixel(i, (g, r, g))

    def effect_aurora_fast(self, speed=0.8):
        """Dynamic Solar Storm Aurora"""
        t = time.time() * speed
        for i in range(self.num_pixels):
            # Faster, more turbulent waves
            w1 = math.sin(i * 0.15 + t)
            w2 = math.cos(i * 0.3 - t * 1.2)
            mix = (w1 + w2) / 2.0
            
            # Shift towards Red/Pink/Purple (Active Aurora)
            hue = 0.8 + mix * 0.2 # Purple to Red range
            sat = 0.9
            val = 0.5 + mix * 0.5
            self.set_pixel(i, hsv_to_rgb(hue, sat, val))
