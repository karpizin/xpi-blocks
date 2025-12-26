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
