import random
import math
import time

class SpecialEffectsMixin:
    def effect_matrix(self, speed=1.0):
        # Green digital rain
        self.fade_to_black(70) # Fast decay for "tail"
        for i in range(self.num_pixels):
            # Chance to spawn a "drop head"
            if random.random() < (0.02 * speed):
                self.set_pixel(i, (0, 255, 0)) # Bright green head
            
        # Optional: shift logic? Rain usually falls. 
        # For a strip, we can just use random sparkles + decay
        # For better look, let's drift pixels
        for i in range(self.num_pixels - 1, 0, -1):
            if self.pixels[i-1] == (0, 255, 0):
                self.set_pixel(i, (0, 150, 0)) # Dimming tail
                self.set_pixel(i-1, (0,0,0))

    def effect_dna(self, speed=2.0):
        # Two intertwining sine waves
        t = time.time() * speed
        self.clear()
        for i in range(self.num_pixels):
            # Wave 1 (Blue)
            w1 = math.sin(i * 0.5 + t)
            if w1 > 0.8: self.set_pixel(i, (0, 0, 255))
            
            # Wave 2 (Red)
            w2 = math.sin(i * 0.5 + t + math.pi)
            if w2 > 0.8: self.set_pixel(i, (255, 0, 0))

    def effect_radar(self, color=(0, 255, 0), speed=10.0):
        # Sweeping beam with tail
        self.step += speed
        pos = int(self.step) % self.num_pixels
        self.fade_to_black(100)
        self.set_pixel(pos, color)

    def effect_tv_static(self, speed=1.0):
        # Black/White/Grey noise
        for i in range(self.num_pixels):
            val = random.randint(0, 255)
            # Thresholding for contrast
            if val > 200: self.set_pixel(i, (255, 255, 255))
            elif val > 100: self.set_pixel(i, (100, 100, 100))
            else: self.set_pixel(i, (0, 0, 0))

    def effect_heartbeat(self, color=(255, 0, 0), speed=1.0):
        # ECG P-QRS-T wave traveling
        self.step += speed * 2
        pos = int(self.step) % self.num_pixels
        self.clear()
        
        # Draw the pulse shape
        # QRS complex (the sharp peak)
        self.set_pixel(pos, color)
        self.set_pixel((pos-1)%self.num_pixels, (int(color[0]*0.5), 0, 0))
        # P and T waves (small bumps)
        self.set_pixel((pos-5)%self.num_pixels, (int(color[0]*0.2), 0, 0))
        self.set_pixel((pos+3)%self.num_pixels, (int(color[0]*0.2), 0, 0))

    def effect_morse_code(self, color=(255, 255, 255), speed=1.0):
        # Flashes "XPI" in Morse: -..-  .--.  ..
        # Total duration based on speed
        pattern = [3,1,1,3, 0, 1,3,3,1, 0, 1,1] # 3=dash, 1=dot, 0=space
        t = int(time.time() * 5 * speed) % (len(pattern) * 2)
        idx = t // 2
        if t % 2 == 0 and idx < len(pattern):
            if pattern[idx] > 0: self.fill(color)
            else: self.clear()
        else:
            self.clear()

    def effect_off(self):
        self.clear()
