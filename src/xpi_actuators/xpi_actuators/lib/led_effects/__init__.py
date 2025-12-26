import inspect
from .base import EffectBase
from .basic import BasicEffectsMixin
from .rainbows import RainbowEffectsMixin
from .chases import ChaseEffectsMixin

class LedEffects(EffectBase, BasicEffectsMixin, RainbowEffectsMixin, ChaseEffectsMixin):
    def __init__(self, num_pixels):
        super().__init__(num_pixels)

    def update(self, effect_name, color, speed):
        """
        Main update loop. 
        effect_name: str
        color: tuple (r,g,b)
        speed: float
        """
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
            # Fallback to solid color if effect not found
            self.effect_solid(color)
            
        return self.pixels
