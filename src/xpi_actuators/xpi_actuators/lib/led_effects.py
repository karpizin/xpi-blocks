import time
import math
import random

def wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return (pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return (0, pos * 3, 255 - pos * 3)

def hsv_to_rgb(h, s, v):
    """Convert HSV (0-1, 0-1, 0-1) to RGB (0-255, 0-255, 0-255)."""
    if s == 0.0: return (int(v*255), int(v*255), int(v*255))
    i = int(h*6.0)
    f = (h*6.0) - i
    p, q, t = v*(1.0 - s), v*(1.0 - s*f), v*(1.0 - s*(1.0-f))
    p, q, t = int(p*255), int(q*255), int(t*255)
    v = int(v*255)
    i %= 6
    if i == 0: return (v, t, p)
    if i == 1: return (q, v, p)
    if i == 2: return (p, v, t)
    if i == 3: return (p, q, v)
    if i == 4: return (t, p, v)
    return (v, p, q)

class LedEffects:
    def __init__(self, num_pixels):
        self.num_pixels = num_pixels
        self.pixels = [(0, 0, 0)] * num_pixels
        self.step = 0
        self.last_update = 0
        
        # State variables for effects
        self.pos = 0
        self.direction = 1
        self.color_idx = 0
        self.fade_val = 0
        self.balls = [] # For bouncing balls
        
    def clear(self):
        self.pixels = [(0, 0, 0)] * self.num_pixels

    def set_pixel(self, i, color):
        if 0 <= i < self.num_pixels:
            self.pixels[i] = color

    def fill(self, color):
        self.pixels = [color] * self.num_pixels

    def fade_to_black(self, scale=10):
        """Dim all pixels by a scaling factor."""
        for i in range(self.num_pixels):
            r, g, b = self.pixels[i]
            r = 0 if r <= 10 else int(r * (255 - scale) / 255)
            g = 0 if g <= 10 else int(g * (255 - scale) / 255)
            b = 0 if b <= 10 else int(b * (255 - scale) / 255)
            self.pixels[i] = (r, g, b)

    # --- Group 1: Basic & Static ---
    
    def effect_solid(self, color=(255, 0, 0), speed=0):
        self.fill(color)

    def effect_blink(self, color=(255, 0, 0), speed=1.0):
        # speed: Hz
        if (time.time() * speed) % 2 < 1:
            self.fill(color)
        else:
            self.fill((0, 0, 0))

    def effect_breathe(self, color=(255, 0, 0), speed=0.5):
        # speed: Hz of full cycle
        val = (math.exp(math.sin(time.time() * speed * math.pi)) - 0.367879441) * 108.0
        scale = val / 255.0
        r, g, b = color
        self.fill((int(r*scale), int(g*scale), int(b*scale)))

    def effect_strobe(self, color=(255, 255, 255), speed=10.0):
        # speed: Hz
        if (time.time() * speed) % 2 < 0.5: # Short flash
            self.fill(color)
        else:
            self.fill((0, 0, 0))

    def effect_fade_in(self, color=(255, 0, 0), speed=1.0):
        # speed: duration in seconds
        if self.step == 0: self.last_update = time.time()
        elapsed = time.time() - self.last_update
        scale = min(1.0, elapsed / max(0.1, speed))
        r, g, b = color
        self.fill((int(r*scale), int(g*scale), int(b*scale)))
        self.step = 1 # Mark as started

    def effect_fade_out(self, color=(255, 0, 0), speed=1.0):
        # speed: duration in seconds
        if self.step == 0: self.last_update = time.time()
        elapsed = time.time() - self.last_update
        scale = max(0.0, 1.0 - (elapsed / max(0.1, speed)))
        r, g, b = color
        self.fill((int(r*scale), int(g*scale), int(b*scale)))
        self.step = 1

    def effect_alternating(self, color=(255, 0, 0), speed=1.0):
        # speed: swap frequency (Hz)
        phase = int(time.time() * speed) % 2
        for i in range(self.num_pixels):
            if (i % 2) == phase:
                self.set_pixel(i, color)
            else:
                self.set_pixel(i, (0, 0, 0))

    # --- Group 2: Rainbows ---

    def effect_static_rainbow(self):
        for i in range(self.num_pixels):
            idx = int(i * 256 / self.num_pixels) & 255
            self.set_pixel(i, wheel(idx))

    def effect_rainbow_cycle(self, speed=10.0):
        # speed: shift rate
        self.step += speed
        for i in range(self.num_pixels):
            idx = int((i * 256 / self.num_pixels) + self.step) & 255
            self.set_pixel(i, wheel(idx))

    def effect_rainbow_breathe(self, speed=0.2):
        # speed: cycle speed
        hue = (time.time() * speed) % 1.0
        rgb = hsv_to_rgb(hue, 1.0, 1.0)
        self.fill(rgb)

    def effect_rainbow_chase(self, speed=5.0):
        self.step += speed
        offset = int(self.step) % self.num_pixels
        self.clear()
        for i in range(self.num_pixels):
             idx = int((i + self.step) * 5) & 255
             if (i + offset) % 3 == 0:
                 self.set_pixel(i, wheel(idx))

    def effect_glitter_rainbow(self, speed=10.0):
        self.effect_rainbow_cycle(speed)
        if random.random() < 0.1:
            idx = random.randint(0, self.num_pixels - 1)
            self.set_pixel(idx, (255, 255, 255))
                 
    # --- Group 3: Chases ---

    def effect_color_wipe(self, color=(0, 0, 255), speed=20.0):
        self.step += speed
        idx = int(self.step) % (self.num_pixels * 2) # Cycle * 2 to clear
        
        if idx < self.num_pixels:
            self.set_pixel(idx, color)
        else:
            # Can implement clearing or secondary color wipe here. 
            # Standard wipe just fills. Let's make it fill then clear.
             self.set_pixel(idx - self.num_pixels, (0,0,0))

    def effect_theater_chase(self, color=(255, 0, 0), speed=10.0):
        self.step += speed
        offset = int(self.step) % 3
        self.clear()
        for i in range(0, self.num_pixels, 3):
            if i + offset < self.num_pixels:
                self.set_pixel(i + offset, color)

    def effect_larson_scanner(self, color=(255, 0, 0), speed=30.0):
        # Cylon eye
        self.step += speed
        pos = int(self.step) % (self.num_pixels * 2 - 2)
        if pos >= self.num_pixels:
            pos = (self.num_pixels * 2 - 2) - pos
        
        self.fade_to_black(150) # Trail
        self.set_pixel(pos, color)
        
    # --- Group 4: Sparkles ---
    
    def effect_sparkle(self, color=(255, 255, 255), speed=1.0):
        # speed defines randomness density
        self.fade_to_black(50)
        if random.random() < (speed * 0.1):
            idx = random.randint(0, self.num_pixels - 1)
            self.set_pixel(idx, color)

    def effect_snow_sparkle(self, color=(100, 100, 100), speed=0.5):
        # Background grey, white sparkles
        self.fill(color)
        if random.random() < (speed * 0.1):
            idx = random.randint(0, self.num_pixels - 1)
            self.set_pixel(idx, (255, 255, 255))

    # --- Group 5: Fire ---

    def effect_fire(self, speed=1.0):
        # Simplistic Fire
        # Cooling
        for i in range(self.num_pixels):
            cooldown = random.randint(0, int(((50 * 10) / self.num_pixels) + 2))
            r, g, b = self.pixels[i]
            g = max(0, g - cooldown)
            r = max(0, r - cooldown) 
            self.pixels[i] = (r, g, 0) # Fire is red/yellow
            
        # Drifting
        for i in range(self.num_pixels - 1, 2, -1):
             self.pixels[i] = self.pixels[i - 1] # Simple shift up
             
        # Igniting
        if random.random() < 0.5:
             y = random.randint(0, 7)
             if y < self.num_pixels:
                # Add heat
                r, g, b = self.pixels[y]
                self.pixels[y] = (min(255, r + 160), min(255, g + 160), 0)

    # --- Group 6: Utility ---
    
    def effect_police(self, speed=5.0):
        # Red / Blue strobe
        step = int(time.time() * speed) % 2
        self.clear()
        mid = self.num_pixels // 2
        if step == 0:
            for i in range(mid): self.set_pixel(i, (255, 0, 0))
        else:
            for i in range(mid, self.num_pixels): self.set_pixel(i, (0, 0, 255))
    
    def effect_progress_bar(self, color=(0, 255, 0), speed=0.0):
        # Speed is used as Percentage (0-100)
        percent = max(0, min(100, speed))
        fill_up_to = int((percent / 100.0) * self.num_pixels)
        self.clear()
        for i in range(fill_up_to):
            self.set_pixel(i, color)

    # --- Mapping logic ---
    def update(self, effect_name, color, speed):
        """
        Main update loop. 
        effect_name: str
        color: tuple (r,g,b)
        speed: float (generic parameter, meaning depends on effect)
        """
        import inspect
        
        method_name = f"effect_{effect_name}"
        if hasattr(self, method_name):
            method = getattr(self, method_name)
            sig = inspect.signature(method)
            params = sig.parameters
            
            kwargs = {}
            if 'color' in params:
                kwargs['color'] = color
            if 'speed' in params:
                kwargs['speed'] = speed
                
            method(**kwargs)
        else:
            # Fallback
            self.effect_solid(color)
            
        return self.pixels

    def reset_state(self):
        """Call this when changing effects to clear internal counters."""
        self.step = 0
        self.last_update = time.time()
        self.clear()
