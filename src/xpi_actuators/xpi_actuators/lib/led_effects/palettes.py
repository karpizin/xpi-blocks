import time
import math

class PaletteEffectsMixin:
    def _apply_palette(self, colors, speed):
        # colors: list of (r,g,b)
        self.step += speed
        offset = int(self.step) % len(colors)
        
        for i in range(self.num_pixels):
            # Scale pixel index to palette length
            idx = (int(i * len(colors) / self.num_pixels) + offset) % len(colors)
            self.set_pixel(i, colors[idx])

    def effect_cyberpunk(self, speed=1.0):
        # Neon Pink and Cyan
        palette = [(255, 0, 255), (0, 255, 255), (100, 0, 255)]
        self._apply_palette(palette, speed)

    def effect_halloween(self, speed=1.0):
        # Orange and Purple
        palette = [(255, 100, 0), (128, 0, 128), (50, 20, 0)]
        self._apply_palette(palette, speed)

    def effect_christmas(self, speed=1.0):
        # Red and Green
        palette = [(255, 0, 0), (0, 255, 0), (255, 255, 255)]
        self._apply_palette(palette, speed)

    def effect_ocean(self, speed=0.5):
        # Blues and Seafoam
        palette = [(0, 0, 255), (0, 128, 255), (0, 255, 128), (0, 50, 150)]
        self._apply_palette(palette, speed)

    def effect_sunset(self, speed=0.5):
        # Purple -> Red -> Orange -> Yellow
        palette = [(128, 0, 128), (255, 0, 0), (255, 128, 0), (255, 255, 0)]
        self._apply_palette(palette, speed)

    def effect_party(self, speed=2.0):
        # Vibrant random assortment
        palette = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]
        self._apply_palette(palette, speed)

    def effect_heatmap(self, speed=1.0):
        # Blue -> Green -> Yellow -> Red
        palette = [(0,0,255), (0,255,0), (255,255,0), (255,0,0)]
        self._apply_palette(palette, speed)
