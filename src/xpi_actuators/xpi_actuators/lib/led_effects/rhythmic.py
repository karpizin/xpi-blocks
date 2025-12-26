import time
import math
import random
from .base import wheel, hsv_to_rgb

class RhythmicEffectsMixin:
    def _get_bpm_data(self, bpm):
        if bpm <= 0: bpm = 120
        beat_duration = 60.0 / bpm
        current_time = time.time()
        # Progress within current beat (0.0 to 1.0)
        progress = (current_time % beat_duration) / beat_duration
        # Beat counter (for 4/4 time)
        beat_num = int(current_time / beat_duration) % 4
        return progress, beat_num

    def effect_beat_pulse(self, color=(255, 0, 0), speed=120.0):
        prog, _ = self._get_bpm_data(speed)
        # Sinusoidal pulse
        scale = (math.sin(prog * math.pi)) # Peaks at 0.5
        r, g, b = color
        self.fill((int(r*scale), int(g*scale), int(b*scale)))

    def effect_beat_blink(self, color=(255, 0, 0), speed=120.0):
        prog, _ = self._get_bpm_data(speed)
        if prog < 0.5:
            self.fill(color)
        else:
            self.clear()

    def effect_beat_strobe(self, color=(255, 255, 255), speed=120.0):
        prog, _ = self._get_bpm_data(speed)
        if prog < 0.1: # Very sharp
            self.fill(color)
        else:
            self.clear()

    def effect_rhythmic_chase(self, color=(0, 255, 0), speed=120.0):
        _, beat_count = self._get_bpm_data(speed)
        self.clear()
        for i in range(self.num_pixels):
            if (i + beat_count) % 4 == 0:
                self.set_pixel(i, color)

    def effect_bass_kick(self, speed=120.0):
        prog, beat_num = self._get_bpm_data(speed)
        if prog < 0.2:
            if beat_num == 0:
                self.fill((255, 0, 0)) # Red on 1
            else:
                self.fill((0, 0, 255)) # Blue on 2,3,4
        else:
            self.fade_to_black(100)

    def effect_metronome(self, color=(255, 255, 255), speed=120.0):
        prog, beat_num = self._get_bpm_data(speed)
        # 0->1 on beat 0,1; 1->0 on beat 2,3
        total_prog = (beat_num + prog) / 4.0
        pos = int(math.sin(total_prog * 2 * math.pi) * (self.num_pixels-1) * 0.5 + (self.num_pixels-1) * 0.5)
        self.clear()
        self.set_pixel(pos, color)

    def effect_center_out_beat(self, color=(255, 0, 255), speed=120.0):
        prog, _ = self._get_bpm_data(speed)
        mid = self.num_pixels // 2
        dist = int(prog * mid)
        self.clear()
        for i in range(dist):
            self.set_pixel(mid + i, color)
            self.set_pixel(mid - i, color)

    def effect_rhythmic_rainbow(self, speed=120.0):
        _, beat_num = self._get_bpm_data(speed)
        for i in range(self.num_pixels):
            idx = (int(i * 256 / self.num_pixels) + beat_num * 64) & 255
            self.set_pixel(i, wheel(idx))

    def effect_side_to_side(self, color=(0, 255, 255), speed=120.0):
        prog, _ = self._get_bpm_data(speed)
        pos = int(prog * (self.num_pixels - 1))
        self.clear()
        self.set_pixel(pos, color)

    def effect_rhythmic_alternating(self, color=(255, 255, 0), speed=120.0):
        _, beat_num = self._get_bpm_data(speed)
        phase = beat_num % 2
        for i in range(self.num_pixels):
            if (i % 2) == phase:
                self.set_pixel(i, color)
            else:
                self.clear()

    def effect_disco_strobe(self, speed=120.0):
        prog, beat_num = self._get_bpm_data(speed)
        if prog < 0.1:
            # New color every beat
            self.fill(wheel((beat_num * 60 + int(time.time()*10)) & 255))
        else:
            self.clear()

    def effect_heartbeat_bpm(self, color=(255, 0, 0), speed=60.0):
        prog, _ = self._get_bpm_data(speed)
        # Double pulse logic
        val = 0
        if 0.1 < prog < 0.2: val = (prog - 0.1) * 10
        elif 0.3 < prog < 0.6: val = 1.0 - (prog - 0.3) * 3.3
        
        r, g, b = color
        self.fill((int(r*val), int(g*val), int(b*val)))

    def effect_bpm_scanner(self, color=(255, 0, 0), speed=120.0):
        prog, _ = self._get_bpm_data(speed)
        pos = int(prog * (self.num_pixels - 1))
        self.fade_to_black(100)
        self.set_pixel(pos, color)

    def effect_rhythmic_fade(self, color=(255, 255, 255), speed=120.0):
        prog, _ = self._get_bpm_data(speed)
        scale = 1.0 - prog
        r, g, b = color
        self.fill((int(r*scale), int(g*scale), int(b*scale)))

    def effect_energy_bar(self, color=(0, 255, 0), speed=120.0):
        prog, beat_num = self._get_bpm_data(speed)
        total_slots = 4
        current_fill = (beat_num + prog) / total_slots
        limit = int(current_fill * self.num_pixels)
        self.clear()
        for i in range(limit):
            self.set_pixel(i, color)

    def effect_rhythmic_glitter(self, speed=120.0):
        prog, _ = self._get_bpm_data(speed)
        self.fade_to_black(40)
        if prog < 0.2:
            for _ in range(5):
                self.set_pixel(random.randint(0, self.num_pixels-1), (255, 255, 255))

    def effect_color_switch(self, speed=120.0):
        _, beat_num = self._get_bpm_data(speed)
        self.fill(wheel((beat_num * 64) & 255))

    def effect_bpm_snake(self, color=(0, 255, 0), speed=120.0):
        prog, beat_num = self._get_bpm_data(speed)
        # Moves 4 pixels per beat
        base_pos = (beat_num * 4 + int(prog * 4)) % self.num_pixels
        self.clear()
        for i in range(4):
            self.set_pixel((base_pos + i) % self.num_pixels, color)

    def effect_vu_meter_beat(self, color=(255, 255, 0), speed=120.0):
        prog, _ = self._get_bpm_data(speed)
        val = 1.0 - prog
        limit = int(val * self.num_pixels)
        self.clear()
        for i in range(limit):
            self.set_pixel(i, color)

    def effect_rhythmic_bounce(self, color=(255, 255, 255), speed=120.0):
        prog, _ = self._get_bpm_data(speed)
        # Parabolic bounce
        h = 4 * prog * (1 - prog) # 0 to 1
        pos = int(h * (self.num_pixels - 1))
        self.clear()
        self.set_pixel(pos, color)
