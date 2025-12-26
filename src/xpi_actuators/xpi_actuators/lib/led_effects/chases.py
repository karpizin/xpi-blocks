import random

class ChaseEffectsMixin:
    def effect_color_wipe(self, color=(0, 0, 255), speed=20.0):
        self.step += speed
        idx = int(self.step) % (self.num_pixels * 2)
        if idx < self.num_pixels:
            self.set_pixel(idx, color)
        else:
             self.set_pixel(idx - self.num_pixels, (0,0,0))

    def effect_reverse_wipe(self, color=(0, 0, 255), speed=20.0):
        self.step += speed
        idx = int(self.step) % (self.num_pixels * 2)
        if idx < self.num_pixels:
            self.set_pixel(self.num_pixels - 1 - idx, color)
        else:
            self.set_pixel(self.num_pixels - 1 - (idx - self.num_pixels), (0,0,0))

    def effect_theater_chase(self, color=(255, 0, 0), speed=10.0):
        self.step += speed
        offset = int(self.step) % 3
        self.clear()
        for i in range(0, self.num_pixels, 3):
            if i + offset < self.num_pixels:
                self.set_pixel(i + offset, color)

    def effect_bounce(self, color=(255, 0, 0), speed=20.0):
        self.step += speed
        pos = int(self.step) % (self.num_pixels * 2 - 2)
        if pos >= self.num_pixels:
            pos = (self.num_pixels * 2 - 2) - pos
        self.clear()
        self.set_pixel(pos, color)

    def effect_larson_scanner(self, color=(255, 0, 0), speed=30.0):
        self.step += speed
        pos = int(self.step) % (self.num_pixels * 2 - 2)
        if pos >= self.num_pixels:
            pos = (self.num_pixels * 2 - 2) - pos
        self.fade_to_black(150)
        self.set_pixel(pos, color)
