"""TB6612 driver: which GPIOs it claims, STBY handling, single-motor setups."""
import pytest

pytest.importorskip('gpiozero')
from gpiozero import Device  # noqa: E402
from std_msgs.msg import Float32  # noqa: E402


@pytest.fixture
def tb6612(ros):
    from xpi_actuators.tb6612_driver_node import TB6612DriverNode

    def make(**params):
        ros.OVERRIDES.update({'mock_hardware': True, **params})
        node = TB6612DriverNode()
        return node, Device.pin_factory
    yield make
    Device.pin_factory = None


def _claimed(factory):
    """GPIO numbers that have a device on them."""
    return {next(n for n in info.names if isinstance(n, int)) for info, pin in factory.pins.items()
            if pin.function == 'output'}


def test_smart_motor_launch_pins(tb6612):
    """The parameters smart_motor.launch.py passes: motor A on 18/23/24, STBY 25, no motor B."""
    node, factory = tb6612(motor_a_pwm_pin=18, motor_a_in1_pin=23, motor_a_in2_pin=24,
                           motor_b_pwm_pin=-1, stby_pin=25)
    assert _claimed(factory) == {18, 23, 24, 25}
    stby = node.stby
    assert stby.value == 1                       # driver enabled while running
    node.motor_a_callback(Float32(data=0.5))
    assert node.motor_a_pwm.value == pytest.approx(0.5) and node.motor_a_in1.value == 1
    node.motor_b_callback(Float32(data=0.5))     # disabled motor: no GPIO, no crash
    node.destroy_node()
    assert stby.closed                           # set LOW, then released


def test_stby_low_on_shutdown(tb6612):
    node, factory = tb6612(stby_pin=25)
    pin = factory.pin(25)
    assert pin.state == 1
    node.destroy_node()
    assert pin.state == 0


def test_defaults_unchanged(tb6612):
    node, factory = tb6612()
    assert _claimed(factory) == {12, 5, 6, 13, 26, 19}
    assert node.stby is None


def test_both_motors_disabled_is_an_error(tb6612):
    with pytest.raises(ValueError, match='nothing to drive'):
        tb6612(motor_a_pwm_pin=-1, motor_b_pwm_pin=-1)
