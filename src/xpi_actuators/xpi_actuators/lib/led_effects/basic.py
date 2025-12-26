import time
import math
import random

class BasicEffectsMixin:
    def effect_solid(self, color=(255, 0, 0), speed=0):
        self.fill(color)

    def effect_blink(self, color=(255, 0, 0), speed=1.0):
        if (time.time() * speed) % 2 < 1:
            self.fill(color)
        else:
            self.fill((0, 0, 0))

    def effect_breathe(self, color=(255, 0, 0), speed=0.5):
        val = (math.exp(math.sin(time.time() * speed * math.pi)) - 0.367879441) * 108.0
        scale = val / 255.0
        r, g, b = color
        self.fill((int(r*scale), int(g*scale), int(b*scale)))

    def effect_strobe(self, color=(255, 255, 255), speed=10.0):
        if (time.time() * speed) % 2 < 0.5:
            self.fill(color)
        else:
            self.fill((0, 0, 0))

    def effect_hyper_strobe(self, color=(255, 255, 255), speed=25.0):
        if random.random() < 0.5:
            self.fill(color)
        else:
            self.fill((0, 0, 0))

    def effect_fade_in(self, color=(255, 0, 0), speed=1.0):
        elapsed = time.time() - self.last_update
        scale = min(1.0, elapsed / max(0.1, speed))
        r, g, b = color
        self.fill((int(r*scale), int(g*scale), int(b*scale)))

    def effect_fade_out(self, color=(255, 0, 0), speed=1.0):
        elapsed = time.time() - self.last_update
        scale = max(0.0, 1.0 - (elapsed / max(0.1, speed)))
        r, g, b = color
        self.fill((int(r*scale), int(g*scale), int(b*scale)))

    def effect_alternating(self, color=(255, 0, 0), speed=1.0):
        phase = int(time.time() * speed) % 2
        for i in range(self.num_pixels):
            if (i % 2) == phase:
                self.set_pixel(i, color)
            else:
                self.set_pixel(i, (0, 0, 0))

    def effect_flash(self, color=(255, 255, 255), speed=2.0):
        if self.step == 0:
            self.fill(color)
            self.step = 1
        else:
            self.fade_to_black(int(speed * 10))
