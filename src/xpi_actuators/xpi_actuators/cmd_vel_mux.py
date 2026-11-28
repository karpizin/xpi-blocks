"""Priority multiplexer for velocity commands, with an emergency-stop latch.

No ROS dependency, so it can be unit-tested directly.

* Every source (RC receiver, gamepad, web joystick, BLE, LLM agent...) is an input with a
  priority and a timeout. The output follows the highest-priority input that has sent a
  command within its timeout; lower ones are ignored meanwhile. Nothing active -> stop.
* The emergency stop is a latch: once engaged, the output is zero whatever the inputs say,
  until it is released explicitly. Releasing it does not let a source carry on where it was:
  each input is ignored until it has sent a stop (a zero command) once, so a joystick held
  forward, or an agent in the middle of a move, cannot drive off the moment the stop is
  released.
"""
import math
import threading
from dataclasses import dataclass

STOP_EPSILON = 1e-3   # |v|, |w| below this count as a zero command


@dataclass
class MuxInput:
    name: str
    topic: str
    priority: int
    timeout: float


def parse_inputs(specs):
    """'name:topic:priority:timeout' strings -> [MuxInput], highest priority first."""
    inputs, names = [], set()
    for spec in specs:
        parts = str(spec).split(':')
        if len(parts) != 4:
            raise ValueError(f"mux input {spec!r}: expected 'name:topic:priority:timeout'")
        name, topic, priority, timeout = parts
        if not name or not topic or name in names:
            raise ValueError(f'mux input {spec!r}: empty or duplicate name/topic')
        try:
            priority, timeout = int(priority), float(timeout)
        except ValueError:
            raise ValueError(f'mux input {spec!r}: priority must be an integer, timeout a number')
        if not (timeout > 0 and math.isfinite(timeout)):
            raise ValueError(f'mux input {spec!r}: timeout must be > 0')
        names.add(name)
        inputs.append(MuxInput(name, topic, priority, timeout))
    if not inputs:
        raise ValueError('cmd_vel mux needs at least one input')
    if len({i.priority for i in inputs}) != len(inputs):
        raise ValueError('cmd_vel mux inputs need distinct priorities')
    return sorted(inputs, key=lambda i: -i.priority)


def _finite(x):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return 0.0
    return x if math.isfinite(x) else 0.0


class TwistMux:
    def __init__(self, inputs):
        self.inputs = list(inputs)          # highest priority first
        self._last = {}                     # name -> (time, v, w)
        self._armed = {i.name: True for i in self.inputs}
        self._lock = threading.Lock()
        self.estop_reason = None            # None = not engaged

    @property
    def estop_engaged(self):
        return self.estop_reason is not None

    def update(self, name, linear_x, angular_z, now):
        v, w = _finite(linear_x), _finite(angular_z)
        with self._lock:
            if not self._armed[name]:
                if abs(v) < STOP_EPSILON and abs(w) < STOP_EPSILON:
                    self._armed[name] = True    # it has sent a stop: listen to it again
                else:
                    return
            self._last[name] = (now, v, w)

    def engage_estop(self, reason):
        with self._lock:
            if self.estop_reason is None:
                self.estop_reason = reason
                return True
            return False

    def release_estop(self):
        """Release the latch; every input must send a stop before it is obeyed again."""
        with self._lock:
            self.estop_reason = None
            self._last.clear()
            for name in self._armed:
                self._armed[name] = False

    def is_armed(self, name):
        return self._armed[name]

    def select(self, now):
        """(active input name or None, linear_x, angular_z) to output now."""
        with self._lock:
            if self.estop_reason is not None:
                return None, 0.0, 0.0
            for inp in self.inputs:
                last = self._last.get(inp.name)
                if last and now - last[0] <= inp.timeout:
                    return inp.name, last[1], last[2]
            return None, 0.0, 0.0
