import random
import time

class SparkleEffectsMixin:
    def effect_sparkle(self, color=(255, 255, 255), speed=1.0):
        # speed: density of sparkles (0.1 to 10.0)
        self.fade_to_black(50)
        if random.random() < (speed * 0.05):
            idx = random.randint(0, self.num_pixels - 1)
            self.set_pixel(idx, color)

    def effect_snow_sparkle(self, color=(100, 100, 100), speed=0.5):
        # color: background color
        # speed: frequency of white flashes
        self.fill(color)
        if random.random() < (speed * 0.1):
            idx = random.randint(0, self.num_pixels - 1)
            self.set_pixel(idx, (255, 255, 255))

    def effect_fireflies(self, color=(200, 255, 0), speed=1.0):
        # color: fly color
        # speed: fade speed
        self.fade_to_black(int(10 * speed))
        if random.random() < 0.05:
            idx = random.randint(0, self.num_pixels - 1)
            self.set_pixel(idx, color)

    def effect_lightning(self, color=(255, 255, 255), speed=1.0):
        # speed: intensity/frequency
        self.fade_to_black(100)
        if random.random() < (0.02 * speed):
            # Flash the whole strip or large segments
            flash_color = tuple(int(c * random.random()) for c in color)
            self.fill(flash_color)
            # Occasional double strike
            if random.random() < 0.3:
                self.last_update = time.time() # Hack to trigger logic if needed

    def effect_twinkle(self, color=(255, 255, 255), speed=1.0):
        # Randomly change brightness of pixels
        for i in range(self.num_pixels):
            if random.random() < (0.05 * speed):
                r, g, b = color
                factor = random.random()
                self.set_pixel(i, (int(r*factor), int(g*factor), int(b*factor)))
        self.fade_to_black(10)
