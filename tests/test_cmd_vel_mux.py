"""cmd_vel mux: priorities, timeouts, emergency-stop latch, re-arming, stop button."""
import pytest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from xpi_actuators.cmd_vel_mux import TwistMux, parse_inputs

INPUTS = parse_inputs(['rc:cmd_vel/rc:100:0.5', 'web:cmd_vel/web:80:0.5', 'agent:cmd_vel/agent:50:1.0'])


def twist(v=0.0, w=0.0):
    t = Twist()
    t.linear.x, t.angular.z = v, w
    return t


# ---------------------------------------------------------------- logic
def test_highest_active_priority_wins():
    mux = TwistMux(INPUTS)
    mux.update('agent', 0.3, 0.0, now=10.0)
    assert mux.select(10.1) == ('agent', 0.3, 0.0)
    mux.update('web', 0.1, 0.5, now=10.2)
    assert mux.select(10.3) == ('web', 0.1, 0.5)      # web outranks the agent
    mux.update('rc', 0.0, 0.0, now=10.3)
    assert mux.select(10.4) == ('rc', 0.0, 0.0)       # RC holding still also wins: operator present


def test_timed_out_input_hands_over():
    mux = TwistMux(INPUTS)
    mux.update('agent', 0.3, 0.0, now=10.0)
    mux.update('web', 0.1, 0.0, now=10.0)
    assert mux.select(10.4)[0] == 'web'
    assert mux.select(10.6)[0] == 'agent'             # web silent > 0.5 s, agent within 1.0 s
    assert mux.select(11.1) == (None, 0.0, 0.0)       # nothing active: stop


def test_estop_latch_and_rearm():
    mux = TwistMux(INPUTS)
    mux.update('web', 0.4, 0.0, now=1.0)
    assert mux.engage_estop('button')
    assert not mux.engage_estop('again')              # already engaged
    mux.update('web', 0.4, 0.0, now=1.1)
    assert mux.select(1.2) == (None, 0.0, 0.0)        # latched: zero whatever the inputs say
    mux.release_estop()
    mux.update('web', 0.4, 0.0, now=1.3)              # still held forward: ignored
    assert mux.select(1.35) == (None, 0.0, 0.0) and not mux.is_armed('web')
    mux.update('web', 0.0, 0.0, now=1.4)              # sends a stop once: armed again
    mux.update('web', 0.2, 0.0, now=1.5)
    assert mux.select(1.6) == ('web', 0.2, 0.0)


def test_non_finite_counts_as_stop():
    mux = TwistMux(INPUTS)
    mux.update('web', float('nan'), float('inf'), now=1.0)
    assert mux.select(1.1) == ('web', 0.0, 0.0)


@pytest.mark.parametrize('specs', [[], ['a:t:1'], ['a:t:x:0.5'], ['a:t:1:0'], ['a:t:1:0.5', 'a:u:2:0.5'],
                                   ['a:t:1:0.5', 'b:u:1:0.5']])
def test_bad_inputs(specs):
    with pytest.raises(ValueError):
        parse_inputs(specs)


# ---------------------------------------------------------------- node
@pytest.fixture
def mux_node(ros):
    from xpi_actuators.cmd_vel_mux_node import CmdVelMuxNode

    def make(**params):
        ros.OVERRIDES.update(params)
        node = CmdVelMuxNode()
        sub = {s.topic: s.callback for s in node.subscriptions_}
        pub = {p.topic: p for p in node.publishers_}
        release = next(s for s in node.services_ if s.srv_name == '~/release_estop').callback
        return node, sub, pub, lambda: release(Trigger.Request(), Trigger.Response())
    yield make
    from gpiozero import Device
    Device.pin_factory = None


def out(pub):
    m = pub['cmd_vel_safe'].messages[-1]
    return m.linear.x, m.angular.z


def test_node_default_inputs_and_output(mux_node):
    node, sub, pub, release = mux_node()
    for topic in ['cmd_vel/rc', 'cmd_vel/gamepad', 'cmd_vel/web', 'cmd_vel/ble', 'cmd_vel/gesture',
                  'cmd_vel/agent', 'cmd_vel/swarm', 'estop']:
        assert topic in sub
    sub['cmd_vel/agent'](twist(0.3))
    sub['cmd_vel/web'](twist(0.1, 0.2))
    node._tick()
    assert out(pub) == (0.1, 0.2)
    assert pub['~/active_input'].messages[-1].data == 'web'


def test_node_estop_topic_latches_until_service(mux_node):
    node, sub, pub, release = mux_node()
    sub['cmd_vel/web'](twist(0.3))
    sub['estop'](Bool(data=True))
    assert out(pub) == (0.0, 0.0)                     # immediate stop, not only at the next tick
    sub['estop'](Bool(data=False))                    # false does not release
    sub['cmd_vel/web'](twist(0.3))
    node._tick()
    assert out(pub) == (0.0, 0.0)
    assert pub['~/estop_active'].messages[-1].data is True
    assert pub['~/active_input'].messages[-1].data == 'estop'
    r = release()
    assert r.success and pub['~/estop_active'].messages[-1].data is False
    sub['cmd_vel/web'](twist(0.3))                    # still pushing forward: ignored
    node._tick()
    assert out(pub) == (0.0, 0.0)
    sub['cmd_vel/web'](twist(0.0))
    sub['cmd_vel/web'](twist(0.3))
    node._tick()
    assert out(pub) == (0.3, 0.0)


def test_node_estop_on_start(mux_node):
    node, sub, pub, release = mux_node(estop_on_start=True)
    sub['cmd_vel/rc'](twist(0.5))
    node._tick()
    assert out(pub) == (0.0, 0.0)
    assert release().success


def _button_factory():
    """Mock pins whose pull-up does not override a switch that already holds the pin LOW."""
    from gpiozero import Device
    from gpiozero.pins.mock import MockFactory, MockPin

    class SwitchPin(MockPin):
        held_low = False

        def _set_pull(self, value):
            super()._set_pull(value)
            if self.held_low:
                self.drive_low()
    Device.pin_factory = MockFactory(pin_class=SwitchPin)
    return Device.pin_factory


def test_stop_button_normally_closed(mux_node):
    """NC button to GND with pull-up: closed (LOW) = run, open (HIGH) = stop. Wire cut = stop."""
    from gpiozero import Device
    _button_factory().pin(21).held_low = True         # button closed before the node starts
    node, sub, pub, release = mux_node(estop_gpio=21, mock_hardware=True)
    assert not node.mux.estop_engaged
    sub['cmd_vel/web'](twist(0.3))
    node._tick()
    assert out(pub) == (0.3, 0.0)
    Device.pin_factory.pin(21).drive_high()           # pressed (or the wire broke)
    assert node.mux.estop_engaged
    assert out(pub) == (0.0, 0.0)
    r = release()
    assert not r.success and 'still pressed' in r.message
    Device.pin_factory.pin(21).drive_low()            # button back
    assert release().success


def test_stop_button_open_at_start_starts_latched(mux_node):
    _button_factory()                                 # NC circuit open (pull-up HIGH): unplugged button
    node, sub, pub, release = mux_node(estop_gpio=21, mock_hardware=True)
    assert node.mux.estop_engaged


def test_destroy_sends_stop(mux_node):
    node, sub, pub, release = mux_node()
    sub['cmd_vel/web'](twist(0.3))
    node._tick()
    node.destroy_node()
    assert out(pub) == (0.0, 0.0)
