"""Regression tests for xpi_commons.i2c_helper.

Sensor nodes branch on ``self.bus.mock_mode``; these tests make sure every bus
returned by ``get_smbus()`` carries that flag and that MockSMBus implements the
smbus2 methods the nodes call. A bus that cannot be opened must raise, never
turn into a MockSMBus that publishes simulated data.
"""
import pytest

from xpi_commons.hardware import HardwareUnavailableError
from xpi_commons.i2c_helper import MockSMBus, get_smbus

from xpi_commons import i2c_helper

SMBUS_METHODS_USED_BY_NODES = [
    'read_byte', 'write_byte',
    'read_byte_data', 'write_byte_data',
    'read_word_data', 'write_word_data',
    'read_i2c_block_data', 'write_i2c_block_data',
    'i2c_rdwr', 'close',
]


class _FakeSMBus:
    def __init__(self, bus_id):
        self.bus_id = bus_id


def test_mock_bus_has_mock_mode_true():
    assert get_smbus(1, mock=True).mock_mode is True


def test_real_bus_has_mock_mode_false(monkeypatch):
    monkeypatch.setattr(i2c_helper, 'SMBus', _FakeSMBus)
    monkeypatch.delenv('XPI_MOCK_I2C', raising=False)
    bus = get_smbus(1, mock=False)
    assert isinstance(bus, _FakeSMBus)
    assert bus.mock_mode is False


@pytest.mark.parametrize('error', [FileNotFoundError('/dev/i2c-1'), PermissionError('/dev/i2c-1')])
def test_unavailable_bus_raises_instead_of_mocking(monkeypatch, error):
    def _fail(bus_id):
        raise error

    monkeypatch.setattr(i2c_helper, 'SMBus', _fail)
    monkeypatch.delenv('XPI_MOCK_I2C', raising=False)
    with pytest.raises(HardwareUnavailableError, match='mock_hardware:=true'):
        get_smbus(1, mock=False)


def test_missing_smbus2_raises_instead_of_mocking(monkeypatch):
    monkeypatch.setattr(i2c_helper, 'SMBus', None)
    monkeypatch.delenv('XPI_MOCK_I2C', raising=False)
    with pytest.raises(HardwareUnavailableError, match='smbus2'):
        get_smbus(1, mock=False)


def test_env_var_forces_mock(monkeypatch):
    monkeypatch.setattr(i2c_helper, 'SMBus', _FakeSMBus)
    monkeypatch.setenv('XPI_MOCK_I2C', '1')
    assert get_smbus(1, mock=False).mock_mode is True


def test_mock_bus_implements_methods_used_by_nodes():
    bus = MockSMBus(1)
    missing = [m for m in SMBUS_METHODS_USED_BY_NODES if not callable(getattr(bus, m, None))]
    assert not missing, f'MockSMBus is missing: {missing}'


def test_mock_block_roundtrip():
    bus = MockSMBus(1)
    bus.write_i2c_block_data(0x38, 0xAC, [0x33, 0x00])
    assert bus.read_i2c_block_data(0x38, 0xAC, 2) == [0x33, 0x00]
    assert bus.read_i2c_block_data(0x38, 0x00, 6) == [0] * 6
