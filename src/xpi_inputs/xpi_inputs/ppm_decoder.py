"""PPM (CPPM) frame decoder: edge timestamps in, normalized channels out. No ROS, no GPIO.

A PPM stream is a train of short fixed-width pulses. The value of channel N is the time
between edge N and edge N+1 (same polarity), typically 1000-2000 us; a long gap (the sync
gap, > ~3 ms) marks the start of a frame:

    sync | ch1 | ch2 | ... | chN | sync | ch1 ...
         ^     ^     ^           ^
         edges (e.g. rising) timestamped by pigpio

Timestamps are pigpio ticks: microseconds in an unsigned 32-bit counter that wraps
about every 72 minutes, so differences are taken modulo 2**32.
"""

TICK_MASK = 0xFFFFFFFF


class PpmDecoder:
    def __init__(self, num_channels=8, sync_gap_us=3000, min_pulse_us=900, max_pulse_us=2100,
                 center_pulse_us=1500):
        if num_channels < 1:
            raise ValueError('num_channels must be >= 1')
        if not min_pulse_us < center_pulse_us < max_pulse_us:
            raise ValueError('need min_pulse_us < center_pulse_us < max_pulse_us')
        if sync_gap_us <= max_pulse_us:
            raise ValueError('sync_gap_us must be longer than max_pulse_us')
        self.num_channels = num_channels
        self.sync_gap_us = sync_gap_us
        self.min_pulse_us = min_pulse_us
        self.max_pulse_us = max_pulse_us
        self.center_pulse_us = center_pulse_us
        self._last_tick = None
        self._widths = []
        self._in_frame = False   # a sync gap was seen and no bad interval since
        self.frames = 0          # complete frames decoded
        self.errors = 0          # frames dropped (bad interval or wrong channel count)

    def normalize(self, width_us):
        """Pulse interval in us -> [-1, 1], center_pulse_us = 0."""
        half_range = (self.max_pulse_us - self.min_pulse_us) / 2.0
        return max(-1.0, min(1.0, (width_us - self.center_pulse_us) / half_range))

    def feed(self, tick_us):
        """Feed the timestamp of one edge. Returns the frame (list of floats) when one completes."""
        frame = None
        if self._last_tick is not None:
            dt = (tick_us - self._last_tick) & TICK_MASK
            if dt >= self.sync_gap_us:
                frame = self._end_frame()
                self._in_frame = True
                self._widths = []
            elif self._in_frame:
                if self.min_pulse_us <= dt <= self.max_pulse_us:
                    self._widths.append(dt)
                    if len(self._widths) > self.num_channels:
                        self._drop()       # more channels than configured: wrong setup or noise
                else:
                    self._drop()           # glitch: ignore everything until the next sync gap
        self._last_tick = tick_us
        return frame

    def _drop(self):
        self.errors += 1
        self._in_frame = False
        self._widths = []

    def _end_frame(self):
        if not self._in_frame:
            return None
        if len(self._widths) != self.num_channels:
            if self._widths:
                self.errors += 1
            return None
        self.frames += 1
        return [self.normalize(w) for w in self._widths]
