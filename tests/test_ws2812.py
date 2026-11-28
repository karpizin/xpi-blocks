"""WS2812 driver: off-target (not a Raspberry Pi) it must fail cleanly, not segfault."""
import os
import subprocess
import sys
import textwrap

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _make(ros, **params):
    from xpi_actuators.ws2812_driver_node import WS2812DriverNode
    ros.OVERRIDES.update({'led_count': 8, **params})
    return WS2812DriverNode()


def test_mock_mode_starts_without_strip(ros):
    node = _make(ros, mock_hardware=True)
    assert node.strip is None


def test_missing_library_is_reported(ros, monkeypatch):
    import xpi_actuators.ws2812_driver_node as mod
    from xpi_commons.hardware import HardwareUnavailableError
    monkeypatch.setattr(mod, 'PixelStrip', None)
    with pytest.raises(HardwareUnavailableError, match='rpi_ws281x'):
        _make(ros, mock_hardware=False)


def test_failed_begin_is_reported_and_strip_discarded(ros, monkeypatch):
    import xpi_actuators.ws2812_driver_node as mod
    from xpi_commons.hardware import HardwareUnavailableError
    discarded = []

    class FailingStrip:
        def __init__(self, *args):
            pass

        def begin(self):
            raise RuntimeError('ws2811_init failed with code -3 (Hardware revision is not supported)')
    monkeypatch.setattr(mod, 'PixelStrip', FailingStrip)
    monkeypatch.setattr(mod, 'discard_strip', discarded.append)
    with pytest.raises(HardwareUnavailableError, match='Hardware revision is not supported'):
        _make(ros, mock_hardware=False)
    assert len(discarded) == 1


@pytest.mark.skipif(os.path.exists('/proc/device-tree/model')
                    and 'Raspberry Pi' in open('/proc/device-tree/model').read(),
                    reason='on a Raspberry Pi begin() may succeed and drive real LEDs')
def test_real_library_off_target_exits_cleanly():
    """The real rpi_ws281x: a failed begin() used to segfault the process at exit."""
    pytest.importorskip('rpi_ws281x')
    code = textwrap.dedent('''
        from rpi_ws281x import PixelStrip
        from xpi_actuators.ws2812_driver_node import discard_strip
        strip = PixelStrip(8, 18, 800000, 10, False, 255, 0)
        try:
            strip.begin()
        except RuntimeError:
            discard_strip(strip)
            print('discarded')
    ''')
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(
        [os.path.join(REPO, 'tests'), os.path.join(REPO, 'src', 'xpi_actuators'),
         os.path.join(REPO, 'src', 'xpi_commons')]))
    # import ros_stub first so the node module imports without ROS 2
    code = 'import ros_stub, os; ros_stub.install(os.path.join(%r, "src"))\n' % REPO + code
    r = subprocess.run([sys.executable, '-c', code], env=env, capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, f'exit code {r.returncode} (-11 = segfault)\n{r.stderr[-500:]}'
    assert 'discarded' in r.stdout
