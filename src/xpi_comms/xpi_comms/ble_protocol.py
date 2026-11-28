"""BLE bridge wire protocol (no ROS, no Bluetooth: unit-testable).

Command characteristic (phone -> robot), UTF-8:
  * JSON object with ``lx`` (linear x) and/or ``az`` (angular z), each clamped to [-1, 1];
    NaN, Infinity and non-numbers mean 0 (stop)
  * anything else is a free-form command, forwarded as a string
Telemetry characteristic (robot -> phone): compact JSON object.
"""
import json
import math

MAX_COMMAND_BYTES = 512  # a GATT write is at most ~512 bytes anyway


def _axis(value):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(v):
        return 0.0
    return max(-1.0, min(1.0, v))


def parse_command(value):
    """Decode a write to the command characteristic.

    Returns ``(text, motion)``: ``text`` is the decoded string (``None`` if undecodable or too
    long), ``motion`` is ``(lx, az)`` for a motion command, else ``None``.
    """
    if not isinstance(value, (bytes, bytearray)) or len(value) > MAX_COMMAND_BYTES:
        return None, None
    try:
        text = bytes(value).decode('utf-8')
    except UnicodeDecodeError:
        return None, None
    try:
        cmd = json.loads(text)
    except ValueError:
        return text, None
    if not isinstance(cmd, dict) or not ('lx' in cmd or 'az' in cmd):
        return text, None
    return text, (_axis(cmd.get('lx', 0.0)), _axis(cmd.get('az', 0.0)))


def encode_telemetry(data):
    """Telemetry dict -> bytes for the notify characteristic (non-finite floats become null)."""
    clean = {k: (None if isinstance(v, float) and not math.isfinite(v) else v) for k, v in data.items()}
    return json.dumps(clean, separators=(',', ':'), allow_nan=False).encode('utf-8')
