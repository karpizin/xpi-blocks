"""Without mock_hardware:=true, a node with no hardware must fail, not fake data.

Every node that has a ``mock_hardware`` parameter is started with it set to
false while the hardware is made unavailable: I2C buses cannot be opened,
serial ports cannot be opened and gpiozero has no pin factory. A node passes if
it refuses to start, or if it starts but neither runs in mock mode nor
publishes anything. Simulated readings that look real (temperatures, IMU data,
RC sticks) must never be published unless mock mode was asked for.
"""
import importlib
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from test_node_smoke import ENTRY_POINTS, SRC, _describe, _is_bug, _run_main  # noqa: E402

SKIP = {
    # rpi_ws281x maps /dev/mem in C and crashes the interpreter off-target
    'xpi_actuators.ws2812_driver_node': 'rpi_ws281x segfaults without a Raspberry Pi',
}


def _source(module):
    path = os.path.join(SRC, module.split('.')[0], *module.split('.')) + '.py'
    return open(path).read() if os.path.exists(path) else ''


# Any other exception at startup is a loud refusal to run, which is what we want.
PROGRAMMING_ERRORS = (AttributeError, NameError, TypeError, KeyError, IndexError,
                      AssertionError, ImportError, UnboundLocalError)

HARDWARE_NODES = sorted({m for (m, _f), _names in ENTRY_POINTS
                         if "'mock_hardware'" in _source(m)})


@pytest.fixture
def no_hardware(monkeypatch):
    from xpi_commons import i2c_helper

    def no_bus(bus_id):
        raise FileNotFoundError(f"[Errno 2] No such file or directory: '/dev/i2c-{bus_id}'")
    monkeypatch.delenv('XPI_MOCK_I2C', raising=False)
    monkeypatch.delenv('GPIOZERO_PIN_FACTORY', raising=False)
    monkeypatch.setattr(i2c_helper, 'SMBus', no_bus)

    try:
        import serial

        class NoPort:
            def __init__(self, port=None, *a, **k):
                raise serial.SerialException(f'could not open port {port}: No such file or directory')
        monkeypatch.setattr(serial, 'Serial', NoPort)
    except ImportError:
        pass

    try:
        from gpiozero import Device
        from gpiozero.exc import BadPinFactory

        def no_factory():
            raise BadPinFactory('Unable to load any default pin factory!')
        monkeypatch.setattr(Device, 'pin_factory', None)
        monkeypatch.setattr(Device, '_default_pin_factory', staticmethod(no_factory))
    except ImportError:
        pass


def test_hardware_nodes_found():
    assert len(HARDWARE_NODES) > 40


@pytest.mark.parametrize('module_name', HARDWARE_NODES)
def test_no_simulated_data_without_mock_flag(ros, no_hardware, module_name):
    if module_name in SKIP:
        pytest.skip(SKIP[module_name])
    ros.OVERRIDES.update({'mock_hardware': False})
    sys.modules.pop(module_name, None)
    try:
        module = importlib.import_module(module_name)
    except BaseException as e:  # noqa: BLE001
        if _is_bug(e):
            pytest.fail(f'import failed: {_describe(e)}', pytrace=False)
        pytest.skip(f'dependency unavailable: {type(e).__name__}: {str(e)[:80]}')

    exc, hung = _run_main(module.main)
    assert not hung, 'main() did not return after shutdown'
    if exc is not None:
        hardware_error = type(exc).__module__.startswith('gpiozero')   # BadPinFactory is an ImportError
        if isinstance(exc, PROGRAMMING_ERRORS) and not hardware_error:
            pytest.fail(f'crashed instead of reporting missing hardware: {_describe(exc)}',
                        pytrace=False)
        return   # refused to start (HardwareUnavailableError, BadPinFactory, ...): correct

    mock_logs = [r['msg'] for r in ros.LOG if 'mock' in r['msg'].lower()]
    published = {p.topic: len(p.messages) for n in ros.NODES for p in n.publishers_ if p.messages}
    assert not mock_logs, f'switched to mock mode on its own: {mock_logs[0]!r}'
    assert not published, f'published without hardware: {published}'
