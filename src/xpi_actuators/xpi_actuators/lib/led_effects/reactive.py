import random
from .base import wheel

class ReactiveEffectsMixin:
    def effect_reactive_flash(self, color=(255, 255, 255), speed=5.0):
        # speed: decay speed
        if self.beat_triggered:
            self.fill(color)
            self.beat_triggered = False # Reset flag
        else:
            self.fade_to_black(int(10 * speed))

    def effect_reactive_random_color(self, speed=5.0):
        if self.beat_triggered:
            self.fade_val = random.randint(0, 255) # Use as color cache
            self.fill(wheel(int(self.fade_val)))
            self.beat_triggered = False
        else:
            self.fade_to_black(int(10 * speed))

    def effect_reactive_burst(self, color=(255, 0, 255), speed=1.0):
        # Explosion from center
        mid = self.num_pixels // 2
        if self.beat_triggered:
            self.step = 1 # Start explosion
            self.beat_triggered = False
        
        if self.step > 0:
            self.clear()
            dist = int(self.step)
            for i in range(dist):
                if mid + i < self.num_pixels: self.set_pixel(mid + i, color)
                if mid - i >= 0: self.set_pixel(mid - i, color)
            
            self.step += 2 * speed # Explosion speed
            if self.step > mid + 1:
                self.step = 0 # End explosion

    def effect_reactive_sparkle(self, color=(255, 255, 255), speed=1.0):
        self.fade_to_black(50)
        if self.beat_triggered:
            # Multi sparkle
            for _ in range(int(5 * self.beat_intensity)):
                idx = random.randint(0, self.num_pixels - 1)
                self.set_pixel(idx, color)
            self.beat_triggered = False

    def effect_reactive_vu(self, color=(0, 255, 0), speed=1.0):
        # speed: fallback decay
        if self.beat_triggered:
            # Level based on intensity
            self.fade_val = self.beat_intensity * self.num_pixels
            self.beat_triggered = False
        
        self.clear()
        for i in range(int(self.fade_val)):
            if i < self.num_pixels:
                self.set_pixel(i, color)
        
        # Smooth gravity fall
        self.fade_val = max(0, self.fade_val - 0.5 * speed)
