import time
import random
from .base import wheel, hsv_to_rgb

class RainbowEffectsMixin:
    def effect_static_rainbow(self):
        for i in range(self.num_pixels):
            idx = int(i * 256 / self.num_pixels) & 255
            self.set_pixel(i, wheel(idx))

    def effect_rainbow_cycle(self, speed=10.0):
        self.step += speed
        for i in range(self.num_pixels):
            idx = int((i * 256 / self.num_pixels) + self.step) & 255
            self.set_pixel(i, wheel(idx))

    def effect_rainbow_breathe(self, speed=0.2):
        hue = (time.time() * speed) % 1.0
        rgb = hsv_to_rgb(hue, 1.0, 1.0)
        self.fill(rgb)

    def effect_rainbow_chase(self, speed=5.0):
        self.step += speed
        offset = int(self.step) % self.num_pixels
        self.clear()
        for i in range(self.num_pixels):
             idx = int((i + self.step) * 5) & 255
             if (i + offset) % 3 == 0:
                 self.set_pixel(i, wheel(idx))

    def effect_glitter_rainbow(self, speed=10.0):
        self.effect_rainbow_cycle(speed)
        if random.random() < 0.1:
            idx = random.randint(0, self.num_pixels - 1)
            self.set_pixel(idx, (255, 255, 255))

    def effect_rainbow_strobe(self, speed=10.0):
        if (time.time() * speed) % 2 < 0.5:
            self.fill(wheel(int(time.time() * 50) & 255))
        else:
            self.fill((0, 0, 0))

    def effect_pastel_rainbow(self, speed=5.0):
        # speed: movement speed
        self.step += speed
        for i in range(self.num_pixels):
            # Calculate hue based on position and step
            hue = ((i / self.num_pixels) + (self.step / 256.0)) % 1.0
            # Convert HSV to RGB with low saturation (0.5) for pastel look
            rgb = hsv_to_rgb(hue, 0.5, 0.8)
            self.set_pixel(i, rgb)

    def effect_neon_rainbow(self, speed=5.0):
        # speed: movement speed
        self.step += speed
        # Defined neon palette: Pink, Cyan, Lime
        palette = [
            (255, 20, 147), # Deep Pink
            (0, 255, 255),   # Cyan
            (50, 205, 50)    # Lime Green
        ]
        
        for i in range(self.num_pixels):
            # Calculate position in the palette loop
            pos = (i / self.num_pixels * 3.0 + (self.step / 20.0)) % 3.0
            idx = int(pos)
            next_idx = (idx + 1) % 3
            frac = pos - idx
            
            # Interpolate between palette colors
            r = int(palette[idx][0] * (1.0 - frac) + palette[next_idx][0] * frac)
            g = int(palette[idx][1] * (1.0 - frac) + palette[next_idx][1] * frac)
            b = int(palette[idx][2] * (1.0 - frac) + palette[next_idx][2] * frac)
            
            self.set_pixel(i, (r, g, b))

    def effect_double_rainbow(self, speed=5.0):
        # speed: movement speed
        self.step += speed
        center = self.num_pixels // 2
        for i in range(center):
            # Left half moving towards center
            idx_left = int((i * 256 / center) + self.step) & 255
            self.set_pixel(i, wheel(idx_left))
            
            # Right half mirror moving towards center
            idx_right = int(((center - i) * 256 / center) + self.step) & 255
            self.set_pixel(self.num_pixels - 1 - i, wheel(idx_right))
            
        # Handle odd pixel count center
        if self.num_pixels % 2 != 0:
            idx_mid = int((center * 256 / center) + self.step) & 255
            self.set_pixel(center, wheel(idx_mid))
