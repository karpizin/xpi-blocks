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

    def effect_twinkle_random(self, speed=1.0):
        # Randomly change brightness of pixels with random colors
        from .base import wheel
        for i in range(self.num_pixels):
            if random.random() < (0.05 * speed):
                self.set_pixel(i, wheel(random.randint(0, 255)))
        self.fade_to_black(10)

    def effect_rain(self, color=(0, 0, 255), speed=1.0):
        # speed: falling speed
        self.fade_to_black(40) # Fading trail
        
        # Shift all pixels forward
        for i in range(self.num_pixels - 1, 0, -1):
            if self.pixels[i-1] != (0,0,0):
                self.pixels[i] = self.pixels[i-1]
                self.pixels[i-1] = (0,0,0)
        
        # Randomly spawn a "drop" at the start
        if random.random() < (0.1 * speed):
            self.set_pixel(0, color)

    def effect_meteor_rain(self, color=(255, 255, 255), speed=1.0):
        # speed: animation speed
        # 1. Fade all pixels slowly to create trail
        self.fade_to_black(60)
        
        # 2. Shift existing pixels
        # Using speed to determine how often we shift
        self.step += speed
        if self.step >= 1.0:
            shift_amount = int(self.step)
            self.step -= shift_amount
            
            for _ in range(shift_amount):
                for i in range(self.num_pixels - 1, 0, -1):
                    if self.pixels[i-1] != (0,0,0):
                        # Move with some intensity loss
                        self.pixels[i] = self.pixels[i-1]
                        self.pixels[i-1] = (0,0,0)
        
        # 3. Spawn meteor
                self.fade_to_black(10)
        
            def effect_storm(self, color=(20, 20, 50), speed=1.0):
                # color: background color (stormy blue/dark)
                # speed: overall storm activity
                t = time.time() * speed
                # Pulsating background
                factor = (math.sin(t) * 0.2 + 0.3)
                self.fill(tuple(int(c * factor) for c in color))
                
                        # Occasional lightning
                        if random.random() < (0.01 * speed):
                            self.fill((255, 255, 255))
                
                    def effect_snowfall(self, color=(255, 255, 255), speed=1.0):
                        # speed: falling speed
                        # We use a simplified stack-up: pixels at the end stay longer
                        for i in range(self.num_pixels - 1, 0, -1):
                            if self.pixels[i-1] != (0,0,0):
                                # If next pixel is empty, move there
                                if self.pixels[i] == (0,0,0):
                                    self.pixels[i] = self.pixels[i-1]
                                    self.pixels[i-1] = (0,0,0)
                                else:
                                    # Next pixel is full, we "stack up" or just stay
                                    pass
                        
                        # Slowly fade the "snow pile" at the end to prevent total fill
                        end_idx = self.num_pixels - 1
                        if self.pixels[end_idx] != (0,0,0):
                            r, g, b = self.pixels[end_idx]
                            self.pixels[end_idx] = (max(0, r-5), max(0, g-5), max(0, b-5))
                
                        # Spawn new snowflake at the top
                        if random.random() < (0.05 * speed):
                            self.set_pixel(0, color)
                        
