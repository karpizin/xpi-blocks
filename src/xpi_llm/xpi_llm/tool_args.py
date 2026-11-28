"""Validation of tool-call arguments coming from an LLM.

Providers disagree on JSON types: Gemini returns every number as a float
(``relay_id: 5.0``), Ollama models often quote values (``"5"``, ``"true"``).
These helpers accept those spellings, and reject anything that is not exactly
an allowed value. A tool call that fails validation does nothing.
"""
import math


def as_int(value, lo, hi):
    """An integer in [lo, hi]: 5, 5.0 or "5" -> 5. Otherwise None (also 5.5, True, NaN)."""
    if isinstance(value, bool):
        return None
    if isinstance(value, str):
        try:
            value = float(value.strip())
        except ValueError:
            return None
    if isinstance(value, float):
        if not math.isfinite(value) or not value.is_integer():
            return None
        value = int(value)
    if not isinstance(value, int):
        return None
    return value if lo <= value <= hi else None


def as_float(value, lo, hi):
    """A finite number in [lo, hi] (numbers or numeric strings). Otherwise None."""
    if isinstance(value, bool):
        return None
    try:
        v = float(value.strip() if isinstance(value, str) else value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(v):
        return None
    return v if lo <= v <= hi else None


def as_bool(value):
    """True/False, or "true"/"false"/"on"/"off" (any case). Otherwise None (also 0/1)."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return {'true': True, 'on': True, 'false': False, 'off': False}.get(value.strip().lower())
    return None


def relay_topic(relay_id):
    """Topic of relay N: /relay_node/cmd for 0 (a relay_node started with `ros2 run`),
    /relay_node_<N>/cmd for the others (start them with --ros-args -r __node:=relay_node_<N>)."""
    return '/relay_node/cmd' if relay_id == 0 else f'/relay_node_{relay_id}/cmd'
