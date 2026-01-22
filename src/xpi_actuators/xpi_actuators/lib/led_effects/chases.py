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

    def effect_theater_chase_rainbow(self, speed=10.0):
        # speed: movement speed
        self.step += speed
        offset = int(self.step) % 3
        self.clear()
        from .base import wheel
        for i in range(0, self.num_pixels, 3):
            if i + offset < self.num_pixels:
                # Rainbow color based on position and animation step
                idx = int((i + self.step) * 5) & 255
                self.set_pixel(i + offset, wheel(idx))

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

    def effect_kitt_scanner(self, color=(255, 0, 0), speed=20.0):
        # speed: movement speed
        self.step += speed
        pos = int(self.step) % (self.num_pixels * 2 - 2)
        if pos >= self.num_pixels:
            pos = (self.num_pixels * 2 - 2) - pos
        
        # Smoother fade for KITT style
        self.fade_to_black(100)
        
        # Wider head (3 pixels)
        self.set_pixel(pos, color)
        # Neighbor pixels at lower brightness
        side_color = (int(color[0] * 0.5), int(color[1] * 0.5), int(color[2] * 0.5))
        self.set_pixel(pos - 1, side_color)
        self.set_pixel(pos + 1, side_color)

    def effect_comet(self, color=(255, 255, 255), speed=20.0):
        # speed: movement speed
        self.step += speed
        pos = int(self.step) % self.num_pixels
        self.fade_to_black(80) # Trailing effect
        self.set_pixel(pos, color)

    def effect_train(self, color=(255, 0, 0), speed=10.0):
        # speed: movement speed
        # Block of 3 pixels
        self.step += speed
        offset = int(self.step) % self.num_pixels
        self.clear()
        for i in range(3):
            self.set_pixel((offset + i) % self.num_pixels, color)

    def effect_snake(self, color=(0, 255, 0), speed=10.0):
        # speed: movement speed
        self.step += speed
        offset = int(self.step) % self.num_pixels
        self.clear()
        # Snake of 4 pixels with fading tail
        for i in range(4):
            pixel_pos = (offset - i) % self.num_pixels
            # Dim tail pixels
            brightness = 1.0 - (i * 0.25)
            c = (int(color[0] * brightness), int(color[1] * brightness), int(color[2] * brightness))
            self.set_pixel(pixel_pos, c)

    def effect_pac_man(self, speed=10.0):
        # speed: movement speed
        self.step += speed
        pos = int(self.step) % self.num_pixels
        self.clear()
        
        # Pacman (Yellow)
        self.set_pixel(pos, (255, 255, 0))
        
        # Ghost (Red) - following at distance of 2
        ghost_pos = (pos - 2) % self.num_pixels
        self.set_pixel(ghost_pos, (255, 0, 0))

    def effect_conveyor_belt(self, color=(255, 255, 255), speed=10.0):
        # speed: movement speed
        self.step += speed
        offset = int(self.step) % 10 # Block size cycle of 10
        self.clear()
        for i in range(self.num_pixels):
            # 5 pixels color, 5 pixels black
            if (i + offset) % 10 < 5:
                self.set_pixel(i, color)

    def effect_dual_scan(self, color=(255, 0, 0), speed=30.0):
        self.step += speed
        pos1 = int(self.step) % (self.num_pixels * 2 - 2)
        if pos1 >= self.num_pixels:
            pos1 = (self.num_pixels * 2 - 2) - pos1
        
        pos2 = self.num_pixels - 1 - pos1
        self.fade_to_black(150)
        self.set_pixel(pos1, color)
        self.set_pixel(pos2, color)

    def effect_marquee(self, color=(255, 255, 0), speed=5.0):
        self.step += speed
        offset = int(self.step) % 3
        self.clear()
        for i in range(self.num_pixels):
            if (i + offset) % 3 == 0:
                self.set_pixel(i, color)

    def effect_wipe_random(self, speed=20.0):
        if int(self.step) == 0:
            self.fade_val = random.randint(0, 255) # Reusing fade_val as color cache
        
        c = wheel(int(self.fade_val))
        self.effect_color_wipe(color=c, speed=speed)
        
        # Reset step if cycle complete to trigger new color
        if int(self.step) >= self.num_pixels * 2:
            self.step = 0

