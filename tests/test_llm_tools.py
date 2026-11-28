"""LLM tool execution: what reaches the actuators for a given model answer.

The LLM is a fake that returns a fixed (text, tool_call); the nodes run on the
ROS stub, so the test sees exactly the messages published to relays, PCA9685,
motors and DAC.
"""
import math
import threading

import pytest
from std_msgs.msg import String

from xpi_llm.tool_args import as_bool, as_float, as_int


class FakeLLM:
    def __init__(self, *replies, delay=0.0):
        self.replies = list(replies)
        self.prompts = []
        self.delay = delay

    def generate(self, prompt, tools=None, **kwargs):
        self.prompts.append(prompt)
        if self.delay:
            threading.Event().wait(self.delay)
        return self.replies.pop(0) if self.replies else ('', {})

    def get_model_name(self):
        return 'fake'


def call(name, **arguments):
    return '', {'name': name, 'arguments': arguments}


def _pub(node, topic):
    return next(p for p in node.publishers_ if p.topic == topic)


def _sub(node, topic):
    return next(s for s in node.subscriptions_ if s.topic == topic)


# ---------------------------------------------------------------- argument coercion
@pytest.mark.parametrize('value,expected', [(5, 5), (5.0, 5), ('5', 5), (' 7 ', 7), (0, 0), (15, 15),
                                            (5.5, None), (16, None), (-1, None), (True, None),
                                            (float('nan'), None), (None, None), ('five', None)])
def test_as_int(value, expected):
    assert as_int(value, 0, 15) == expected


@pytest.mark.parametrize('value,expected', [(True, True), (False, False), ('true', True), ('OFF', False),
                                            (1, None), (0, None), ('yes', None), (None, None)])
def test_as_bool(value, expected):
    assert as_bool(value) == expected


@pytest.mark.parametrize('value,expected', [(0.5, 0.5), ('0.25', 0.25), (1, 1.0), (1.01, None),
                                            (float('inf'), None), (float('nan'), None), (True, None)])
def test_as_float(value, expected):
    assert as_float(value, 0.0, 1.0) == expected


# ---------------------------------------------------------------- tool_calling_node
@pytest.fixture
def tool_node(ros, monkeypatch):
    import xpi_llm.tool_calling_node as mod

    def make(*replies, **params):
        llm = FakeLLM(*replies)
        monkeypatch.setattr(mod, 'llm_client_factory', lambda *a, **k: llm)
        ros.OVERRIDES.update(params)
        node = mod.ToolCallingNode()

        def say(text):
            _sub(node, '~/command').callback(String(data=text))
            node.commands.join()
            return _pub(node, '~/response').messages[-1].data
        return node, llm, say
    return make


def test_relay_with_gemini_float_id(tool_node):
    """Gemini sends relay_id 5.0: it must switch relay 5, not be rejected."""
    node, llm, say = tool_node(call('set_relay_state', relay_id=5.0, state=True))
    assert 'relay 5' in say('turn on relay 5')
    msgs = _pub(node, '/relay_node_5/cmd').messages
    assert [m.data for m in msgs] == [True]
    assert all(not _pub(node, t).messages for t in ['/relay_node/cmd', '/relay_node_4/cmd'])


@pytest.mark.parametrize('args', [dict(relay_id=5.5, state=True), dict(relay_id=16, state=True),
                                  dict(relay_id=5, state=1), dict(relay_id=5), dict(state=True)])
def test_relay_bad_arguments_do_nothing(tool_node, args):
    node, llm, say = tool_node(call('set_relay_state', **args))
    assert 'Invalid' in say('relay')
    assert not any(p.messages for p in node.publishers_ if p.topic.startswith('/relay_node'))


def test_pwm_sets_only_one_channel(tool_node):
    """The other 15 channels must be left alone (NaN), not switched to 0."""
    node, llm, say = tool_node(call('set_pwm_channel', channel_id=3.0, duty_cycle=0.075))
    assert 'channel 3' in say('center servo 3')
    data = list(_pub(node, '/pwm_driver/cmd').messages[-1].data)
    assert len(data) == 4 and data[3] == pytest.approx(0.075)
    assert all(math.isnan(v) for v in data[:3])


def test_pwm_topic_parameter(tool_node):
    node, llm, say = tool_node(call('set_pwm_channel', channel_id=0, duty_cycle=0.5),
                               pwm_topic='/pca9685_node/cmd')
    say('x')
    assert _pub(node, '/pca9685_node/cmd').messages


@pytest.mark.parametrize('args', [dict(channel_id=3, duty_cycle=1.5), dict(channel_id=3, duty_cycle='nan'),
                                  dict(channel_id=20, duty_cycle=0.5)])
def test_pwm_bad_arguments_do_nothing(tool_node, args):
    node, llm, say = tool_node(call('set_pwm_channel', **args))
    assert 'Invalid' in say('x')
    assert not _pub(node, '/pwm_driver/cmd').messages


def test_non_dict_arguments(tool_node):
    node, llm, say = tool_node(('', {'name': 'set_relay_state', 'arguments': '5'}))
    assert 'Invalid arguments' in say('x')


def test_text_reply_is_published(tool_node):
    node, llm, say = tool_node(('I can only switch relays and PWM channels.', {}))
    assert say('make coffee') == 'I can only switch relays and PWM channels.'


def test_commands_run_one_at_a_time_in_order(tool_node, monkeypatch):
    import xpi_llm.tool_calling_node as mod
    llm = FakeLLM(call('set_relay_state', relay_id=1, state=True),
                  call('set_relay_state', relay_id=1, state=False), delay=0.02)
    monkeypatch.setattr(mod, 'llm_client_factory', lambda *a, **k: llm)
    node = mod.ToolCallingNode()
    active, peak = [0], [0]
    real = llm.generate

    def counting(*a, **k):
        active[0] += 1
        peak[0] = max(peak[0], active[0])
        try:
            return real(*a, **k)
        finally:
            active[0] -= 1
    llm.generate = counting
    cb = _sub(node, '~/command').callback
    cb(String(data='on'))
    cb(String(data='off'))
    node.commands.join()
    assert peak[0] == 1
    assert [m.data for m in _pub(node, '/relay_node_1/cmd').messages] == [True, False]


# ---------------------------------------------------------------- PCA9685 driver
def test_pca9685_nan_leaves_channel_unchanged(ros):
    from xpi_actuators.pca9685_node import PCA9685Node
    from std_msgs.msg import Float32MultiArray
    ros.OVERRIDES.update({'mock_hardware': True})
    node = PCA9685Node()
    writes = []
    node.set_pwm = lambda ch, on, off: writes.append(ch)
    node.cmd_callback(Float32MultiArray(data=[float('nan'), float('nan'), float('nan'), 0.075]))
    assert writes == [3]


# ---------------------------------------------------------------- mcp_agent_node
@pytest.fixture
def agent(ros, monkeypatch):
    import xpi_llm.mcp_agent_node as mod

    def make(*replies, **params):
        llm = FakeLLM(*replies)
        monkeypatch.setattr(mod, 'llm_client_factory', lambda *a, **k: llm)
        ros.OVERRIDES.update(params)
        node = mod.MCPAgentNode()

        def say(text):
            _sub(node, '~/command').callback(String(data=text))
            node.commands.join()
            return _pub(node, '~/response').messages[-1].data
        return node, llm, say
    return make


def test_agent_control_relay_uses_relay_id(agent):
    node, llm, say = agent(call('control_relay', relay_id=3.0, state=True))
    assert 'Relay 3 -> ON' in say('turn on relay 3')
    assert [m.data for m in _pub(node, '/relay_node_3/cmd').messages] == [True]
    assert not _pub(node, '/relay_node/cmd').messages


@pytest.mark.parametrize('name,args', [
    ('control_relay', dict(relay_id=3, state='maybe')),
    ('control_relay', dict(state=True)),
    ('set_led_effect', dict(effect_id=500)),
    ('set_analog_voltage', dict(voltage=12.0)),
    ('set_analog_voltage', dict(voltage=float('nan'))),
    ('set_analog_voltage', dict()),
])
def test_agent_rejects_bad_arguments(agent, name, args):
    node, llm, say = agent(call(name, **args))
    assert 'Rejected' in say('x')
    for topic in ['/relay_node_3/cmd', '/relay_node/cmd', '/ws2812/effect', '/mcp4725/cmd_voltage']:
        assert not _pub(node, topic).messages


def test_agent_voltage_within_limit(agent):
    node, llm, say = agent(call('set_analog_voltage', voltage='1.5'))
    say('x')
    assert _pub(node, '/mcp4725/cmd_voltage').messages[-1].data == pytest.approx(1.5)


def test_agent_unknown_tool(agent):
    node, llm, say = agent(call('self_destruct'))
    assert 'Unknown tool' in say('x')
