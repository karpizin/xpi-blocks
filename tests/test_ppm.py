"""PPM receiver: frame decoding from edge timestamps, and the node with a fake pigpio."""
import threading
import types

import pytest

from xpi_inputs.ppm_decoder import TICK_MASK, PpmDecoder

SYNC = 9000  # us


def edges(frames, start=1_000_000, sync=SYNC):
    """Edge timestamps for a list of frames (each a list of channel intervals in us)."""
    t = start
    out = [t]
    for frame in frames:
        t += sync
        out.append(t)
        for width in frame:
            t += width
            out.append(t)
    t += sync          # the sync gap after the last frame completes it
    out.append(t)
    return [x & TICK_MASK for x in out]


def decode(decoder, ticks):
    return [f for f in (decoder.feed(t) for t in ticks) if f is not None]


def test_decodes_frames():
    d = PpmDecoder(num_channels=4)
    frames = decode(d, edges([[1500, 1000, 2000, 1750], [1250, 1500, 1500, 1500]]))
    assert frames == [[0.0, -500 / 600, 500 / 600, 250 / 600], [-250 / 600, 0.0, 0.0, 0.0]]
    assert (d.frames, d.errors) == (2, 0)


def test_values_are_clamped():
    d = PpmDecoder(num_channels=2, min_pulse_us=900, max_pulse_us=2100)
    assert decode(d, edges([[900, 2100]])) == [[-1.0, 1.0]]


def test_tick_wraparound():
    d = PpmDecoder(num_channels=3)
    start = TICK_MASK - 2500   # the 32-bit pigpio tick wraps inside the frame
    assert decode(d, edges([[1500, 1600, 1400]], start=start)) == [[0.0, 100 / 600, -100 / 600]]


def test_glitch_drops_frame_until_next_sync():
    d = PpmDecoder(num_channels=3)
    ticks = edges([[1500, 300, 1500], [1500, 1500, 1500]])   # 300 us: noise spike
    assert decode(d, ticks) == [[0.0, 0.0, 0.0]]
    assert d.errors == 1


def test_wrong_channel_count_is_dropped():
    d = PpmDecoder(num_channels=4)
    assert decode(d, edges([[1500, 1500, 1500]])) == []           # too few
    d = PpmDecoder(num_channels=2)
    assert decode(d, edges([[1500, 1500, 1500]])) == []           # too many
    assert d.errors == 1


def test_partial_first_frame_is_ignored():
    d = PpmDecoder(num_channels=2)
    ticks = [100, 1600, 3100] + edges([[1500, 1500]], start=3100)[1:]  # joined mid-frame
    assert decode(d, ticks) == [[0.0, 0.0]]


@pytest.mark.parametrize('kwargs', [
    {'num_channels': 0}, {'min_pulse_us': 1600}, {'sync_gap_us': 2000},
])
def test_bad_config(kwargs):
    with pytest.raises(ValueError):
        PpmDecoder(**kwargs)


# ---------------------------------------------------------------- node with a fake pigpio
class FakePi:
    def __init__(self, connected=True):
        self.connected = connected
        self.modes, self.pulls, self.callbacks, self.stopped = {}, {}, [], False

    def set_mode(self, gpio, mode):
        self.modes[gpio] = mode

    def set_pull_up_down(self, gpio, pud):
        self.pulls[gpio] = pud

    def callback(self, user_gpio, edge=0, func=None):
        cb = types.SimpleNamespace(gpio=user_gpio, edge=edge, func=func, cancelled=False)
        cb.cancel = lambda: setattr(cb, 'cancelled', True)
        self.callbacks.append(cb)
        return cb

    def stop(self):
        self.stopped = True


@pytest.fixture
def ppm(ros, monkeypatch):
    pigpio = pytest.importorskip('pigpio')
    import xpi_inputs.ppm_receiver_node as mod
    pis = []

    def make(connected=True, **params):
        pi = FakePi(connected)
        pis.append(pi)
        monkeypatch.setattr(mod.pigpio, 'pi', lambda *a, **k: pi)
        ros.OVERRIDES.update({'num_channels': 4, **params})
        node = mod.PpmReceiverNode()
        return node, pi, next(p for p in node.publishers_ if p.topic == '~/joy')
    return make, pigpio


def wait(seconds):
    threading.Event().wait(seconds)   # real wait: the ros fixture caps time.sleep


def test_node_decodes_and_publishes(ppm):
    make, pigpio = ppm
    node, pi, joy = make(gpio_pin=22)
    assert pi.modes[22] == pigpio.INPUT and pi.pulls[22] == pigpio.PUD_UP
    cb = pi.callbacks[0]
    assert (cb.gpio, cb.edge) == (22, pigpio.RISING_EDGE)
    for t in edges([[1500, 1000, 2000, 1750]]):
        cb.func(22, 1, t)                     # what pigpio calls from its thread
    node.timer_callback()
    assert list(joy.messages[-1].axes) == pytest.approx([0.0, -500 / 600, 500 / 600, 250 / 600])
    node.destroy_node()
    assert cb.cancelled and pi.stopped


def test_falling_edge_and_no_pull_up(ppm):
    make, pigpio = ppm
    node, pi, joy = make(edge='falling', pull_up=False)
    assert pi.callbacks[0].edge == pigpio.FALLING_EDGE
    assert pi.pulls[17] == pigpio.PUD_OFF


def test_stops_publishing_when_signal_lost(ppm):
    make, pigpio = ppm
    node, pi, joy = make(signal_timeout=0.1)
    for t in edges([[1500] * 4]):
        pi.callbacks[0].func(17, 1, t)
    node.timer_callback()
    assert len(joy.messages) == 1
    wait(0.15)
    node.timer_callback()
    node.timer_callback()
    assert len(joy.messages) == 1
    from ros_stub import rclpy_stub
    assert any('signal lost' in r['msg'] for r in rclpy_stub.LOG)


def test_nothing_published_before_first_frame(ppm):
    make, pigpio = ppm
    node, pi, joy = make()
    node.timer_callback()
    assert joy.messages == []


def test_watchdog_timeout_events_are_ignored(ppm):
    make, pigpio = ppm
    node, pi, joy = make()
    pi.callbacks[0].func(17, pigpio.TIMEOUT, 123)    # pigpio watchdog event, not an edge
    assert node.decoder._last_tick is None


def test_daemon_not_running_is_reported(ppm):
    make, pigpio = ppm
    node, pi, joy = make(connected=False)
    from ros_stub import rclpy_stub
    assert any('pigpiod' in r['msg'] for r in rclpy_stub.LOG if r['level'] == 'error')
    assert node.pi is None
    node.timer_callback()
    node.destroy_node()            # no crash without a connection
