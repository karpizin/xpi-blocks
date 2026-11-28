"""TCA9548A mux: channel selection over the ~/cmd topic, state on ~/current_channel."""
import pytest
from std_msgs.msg import Int32
from xpi_commons.i2c_helper import MockSMBus


def _make(ros, **params):
    from xpi_commons.i2c_mux_node import I2CMuxNode
    ros.OVERRIDES.update({'mock_hardware': True, **params})
    node = I2CMuxNode()
    state = next(p for p in node.publishers_ if p.topic == '~/current_channel')
    cmd = next(s for s in node.subscriptions_ if s.topic == '~/cmd')
    return node, state, cmd


def test_state_is_latched_and_reports_every_command(ros):
    node, state, cmd = _make(ros)
    assert state.qos.durability == ros.DurabilityPolicy.TRANSIENT_LOCAL
    for ch in (3, 9, -1, 7):     # 9 is invalid: the active channel (3) is reported again
        cmd.callback(Int32(data=ch))
    assert [m.data for m in state.messages] == [-1, 3, 3, -1, 7]


def test_writes_control_byte_on_hardware(ros, monkeypatch):
    writes = []

    class Bus(MockSMBus):
        mock_mode = False

        def write_byte(self, addr, value):
            writes.append((addr, value))

    import xpi_commons.i2c_mux_node as mux
    monkeypatch.setattr(mux, 'get_smbus', lambda bus_id, mock=False: Bus(bus_id))
    node, state, cmd = _make(ros, mock_hardware=False, default_channel=2)
    cmd.callback(Int32(data=5))
    node.destroy_node()
    assert writes == [(0x70, 0b100), (0x70, 0b100000), (0x70, 0x00)]   # default 2, then 5, off on exit


def test_unavailable_bus_fails_instead_of_mocking(ros, monkeypatch):
    import xpi_commons.i2c_mux_node as mux
    from xpi_commons.hardware import HardwareUnavailableError

    def broken(bus_id, mock=False):
        raise OSError('no /dev/i2c-1')
    monkeypatch.setattr(mux, 'get_smbus', broken)
    with pytest.raises(HardwareUnavailableError, match='mock_hardware:=true'):
        _make(ros, mock_hardware=False)


def test_destroy_without_bus_does_not_crash(ros):
    node, state, cmd = _make(ros, mock_hardware=True)
    node.bus = None
    node.destroy_node()
