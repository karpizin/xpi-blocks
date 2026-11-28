"""PCA9685 drivers write each channel's ON/OFF registers in one I2C transaction."""
import pytest
from std_msgs.msg import Float32, Float32MultiArray


class RecordingBus:
    def __init__(self):
        self.ops = []

    def write_byte_data(self, addr, reg, value):
        self.ops.append(('byte', addr, reg, value))

    def read_byte_data(self, addr, reg):
        return 0x00

    def write_i2c_block_data(self, addr, reg, data):
        self.ops.append(('block', addr, reg, list(data)))


@pytest.fixture
def bus(monkeypatch):
    b = RecordingBus()
    import xpi_actuators.pca9685_node as servo
    import xpi_actuators.pca9685_motor_node as motor
    monkeypatch.setattr(servo, 'get_smbus', lambda *a, **k: b)
    monkeypatch.setattr(motor, 'get_smbus', lambda *a, **k: b)
    return b


def _led_writes(bus):
    """Writes that touch LEDn registers (0x06..0x45)."""
    return [op for op in bus.ops if 0x06 <= op[2] <= 0x45]


def test_auto_increment_enabled(ros, bus):
    from xpi_actuators.pca9685_node import PCA9685Node
    PCA9685Node()
    assert bus.ops[-1][:3] == ('byte', 0x40, 0x00) and bus.ops[-1][3] & 0x20


@pytest.mark.parametrize('duty, regs', [
    (0.5, [0x00, 0x00, 0xFF, 0x07]),     # off tick 2047
    (0.0, [0x00, 0x00, 0x00, 0x10]),     # full OFF bit
    (1.0, [0x00, 0x10, 0x00, 0x00]),     # full ON bit
])
def test_servo_node_single_block_write(ros, bus, duty, regs):
    from xpi_actuators.pca9685_node import PCA9685Node
    node = PCA9685Node()
    bus.ops.clear()
    node.cmd_callback(Float32MultiArray(data=[float('nan'), duty]))
    assert _led_writes(bus) == [('block', 0x40, 0x06 + 4, regs)]


def test_motor_node_single_block_writes(ros, bus):
    from xpi_actuators.pca9685_motor_node import PCA9685MotorNode
    node = PCA9685MotorNode()
    bus.ops.clear()
    node.motor_a_callback(Float32(data=0.5))
    writes = _led_writes(bus)
    assert writes and all(op[0] == 'block' and len(op[3]) == 4 for op in writes)
