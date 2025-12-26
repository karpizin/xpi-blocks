import time
import math

class UtilityEffectsMixin:
    def effect_battery_charge(self, speed=100.0):
        # speed: percentage (0-100)
        percent = max(0, min(100, speed))
        fill_up_to = int((percent / 100.0) * self.num_pixels)
        
        # Determine color based on percentage
        if percent < 20: color = (255, 0, 0) # Red
        elif percent < 50: color = (255, 255, 0) # Yellow
        else: color = (0, 255, 0) # Green
        
        self.clear()
        for i in range(fill_up_to):
            self.set_pixel(i, color)

    def effect_loading_spinner(self, color=(0, 255, 255), speed=10.0):
        # speed: rotation speed
        self.step += speed
        pos = int(self.step) % self.num_pixels
        self.clear()
        # Draw a segment of 3 pixels
        for i in range(3):
            self.set_pixel((pos + i) % self.num_pixels, color)

    def effect_traffic_light(self):
        self.clear()
        third = self.num_pixels // 3
        for i in range(third): self.set_pixel(i, (255, 0, 0)) # Red
        for i in range(third, 2*third): self.set_pixel(i, (255, 255, 0)) # Yellow
        for i in range(2*third, self.num_pixels): self.set_pixel(i, (0, 255, 0)) # Green

    def effect_ambulance(self, speed=10.0):
        phase = int(time.time() * speed) % 2
        self.clear()
        mid = self.num_pixels // 2
        color = (255, 0, 0) if phase == 0 else (255, 255, 255)
        for i in range(self.num_pixels):
            self.set_pixel(i, color)

    def effect_construction(self, speed=5.0):
        # Rotating orange beacon
        self.step += speed
        pos = int(self.step) % self.num_pixels
        self.clear()
        color = (255, 100, 0)
        # Main beacon + side glow
        self.set_pixel(pos, color)
        self.set_pixel((pos + 1) % self.num_pixels, (100, 40, 0))
        self.set_pixel((pos - 1) % self.num_pixels, (100, 40, 0))

    def effect_error_alert(self, speed=15.0):
        # Fast red pulse
        val = (math.sin(time.time() * speed) + 1.0) / 2.0
        self.fill((int(255 * val), 0, 0))

    def effect_success(self, speed=2.0):
        # Green wipe once then stay solid
        if self.step < self.num_pixels:
            self.step += speed
            for i in range(int(self.step)):
                if i < self.num_pixels:
                    self.set_pixel(i, (0, 255, 0))
        else:
            self.fill((0, 255, 0))
