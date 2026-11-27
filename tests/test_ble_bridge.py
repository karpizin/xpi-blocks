"""BLE bridge: protocol, and the node against a fake bless server (no Bluetooth needed).

test_bless_api_matches checks every BlessServer member the node uses against the installed
bless, so the fake below cannot drift from the real library unnoticed.
"""
import asyncio
import inspect
import math
import threading
import time

import pytest

from xpi_comms import ble_protocol as proto

USED_COROUTINES = ['add_new_service', 'add_new_characteristic', 'start', 'stop', 'is_connected']
USED_METHODS = ['get_characteristic', 'update_value', 'write_request', 'read_request']
USED_PROPERTIES = ['read_request_func', 'write_request_func']


# ---------------------------------------------------------------- protocol
@pytest.mark.parametrize('raw, motion', [
    (b'{"lx": 0.5, "az": -0.25}', (0.5, -0.25)),
    (b'{"lx": 3}', (1.0, 0.0)),
    (b'{"az": -9}', (0.0, -1.0)),
    (b'{"lx": NaN, "az": 0.2}', (0.0, 0.2)),
    (b'{"lx": Infinity}', (0.0, 0.0)),
    (b'{"lx": "fast"}', (0.0, 0.0)),
    (b'{"led": "ON"}', None),
    (b'hello', None),
    (b'[1, 2]', None),
])
def test_parse_command(raw, motion):
    text, got = proto.parse_command(bytearray(raw))
    assert text == raw.decode()
    assert got == motion


def test_parse_command_rejects_garbage():
    assert proto.parse_command(b'\xff\xfe') == (None, None)
    assert proto.parse_command(b'x' * (proto.MAX_COMMAND_BYTES + 1)) == (None, None)
    assert proto.parse_command('not bytes') == (None, None)


def test_encode_telemetry():
    assert proto.encode_telemetry({'bat': 12.4, 'status': 'ok'}) == b'{"bat":12.4,"status":"ok"}'
    assert proto.encode_telemetry({'bat': math.nan}) == b'{"bat":null}'


# ---------------------------------------------------------------- bless API
def test_bless_api_matches():
    bless = pytest.importorskip('bless')
    from bless.backends.server import BaseBlessServer
    for name in USED_COROUTINES:
        assert inspect.iscoroutinefunction(getattr(BaseBlessServer, name)), f'{name} must be a coroutine'
    for name in USED_METHODS:
        fn = getattr(BaseBlessServer, name)
        assert callable(fn) and not inspect.iscoroutinefunction(fn), f'{name} must be a plain method'
    for name in USED_PROPERTIES:
        assert isinstance(getattr(BaseBlessServer, name), property), f'{name} must be a property'
    perms = bless.GATTAttributePermissions
    assert {'write_encryption_required', 'read_encryption_required'} <= {p.name for p in perms}
    params = inspect.signature(BaseBlessServer.add_new_characteristic).parameters
    assert list(params)[1:] == ['service_uuid', 'char_uuid', 'properties', 'value', 'permissions']


class FakeChar:
    def __init__(self, uuid, properties, value, permissions):
        self.uuid = uuid.lower()          # bless reports lower-case UUIDs
        self.properties, self.permissions = properties, permissions
        self.value = value if value is not None else bytearray()


class FakeBlessServer:
    """Same members and call pattern as bless.BlessServer (see test_bless_api_matches)."""
    instances = []

    def __init__(self, name, loop=None, **kwargs):
        self.name, self.loop = name, loop
        self.chars, self.notified, self.started, self.stopped = {}, [], False, False
        self.client_connected = False
        self._callbacks = {}
        FakeBlessServer.instances.append(self)

    read_request_func = property(lambda s: s._callbacks['read'],
                                 lambda s, f: s._callbacks.__setitem__('read', f))
    write_request_func = property(lambda s: s._callbacks['write'],
                                  lambda s, f: s._callbacks.__setitem__('write', f))

    async def add_new_service(self, uuid):
        self.service = uuid

    async def add_new_characteristic(self, service_uuid, char_uuid, properties, value, permissions):
        self.chars[char_uuid.lower()] = FakeChar(char_uuid, properties, value, permissions)

    async def start(self, **kwargs):
        self.started = True
        return True

    async def stop(self):
        self.stopped = True
        return True

    async def is_connected(self):
        return self.client_connected

    def get_characteristic(self, uuid):
        return self.chars.get(uuid.lower())

    def update_value(self, service_uuid, char_uuid):
        self.notified.append((service_uuid, char_uuid, bytes(self.get_characteristic(char_uuid).value)))
        return True

    # What the BlueZ backend does when a phone writes / reads
    def write_request(self, uuid, value):
        self.write_request_func(self.get_characteristic(uuid), value)

    def read_request(self, uuid, options=None):
        return self.read_request_func(self.get_characteristic(uuid))


@pytest.fixture
def bridge(ros, monkeypatch):
    pytest.importorskip('bless')
    import xpi_comms.ble_bridge_node as mod
    FakeBlessServer.instances.clear()
    monkeypatch.setattr(mod, 'BlessServer', FakeBlessServer)
    nodes = []

    def make(**params):
        ros.OVERRIDES.update(params)
        node = mod.BLEBridgeNode()
        nodes.append(node)
        deadline = time.monotonic() + 3
        while not node.ready and time.monotonic() < deadline:
            wait(0.01)
        assert node.ready, 'BLE server did not start'
        return node, FakeBlessServer.instances[-1], mod

    yield make
    for n in nodes:
        n.destroy_node()


def wait(seconds):
    """Real wait (the ros fixture caps time.sleep for fast node start-up)."""
    threading.Event().wait(seconds)


def _on_loop(node, fn, *args):
    """Run a bless callback on the BLE loop, as bless does, and wait for it."""
    async def call():
        return fn(*args)
    return asyncio.run_coroutine_threadsafe(call(), node.loop).result(2)


def test_write_drives_cmd_vel(bridge):
    node, srv, mod = bridge()
    _on_loop(node, srv.write_request, mod.CHAR_COMMAND_UUID, bytearray(b'{"lx": 0.4, "az": 0.1}'))
    twist = node.cmd_vel_pub.messages[-1]
    assert (twist.linear.x, twist.angular.z) == (0.4, 0.1)
    assert node.command_pub.messages[-1].data == '{"lx": 0.4, "az": 0.1}'


def test_text_command_is_forwarded_only(bridge):
    node, srv, mod = bridge()
    _on_loop(node, srv.write_request, mod.CHAR_COMMAND_UUID, bytearray(b'{"led": "ON"}'))
    assert node.command_pub.messages[-1].data == '{"led": "ON"}'
    assert node.cmd_vel_pub.messages == []


def test_encryption_required_by_default(bridge):
    node, srv, mod = bridge()
    perms = mod.GATTAttributePermissions
    assert srv.chars[mod.CHAR_COMMAND_UUID.lower()].permissions == perms.write_encryption_required
    assert srv.chars[mod.CHAR_TELEMETRY_UUID.lower()].permissions == perms.read_encryption_required


def test_encryption_can_be_disabled(bridge):
    node, srv, mod = bridge(require_encryption=False)
    perms = mod.GATTAttributePermissions
    assert srv.chars[mod.CHAR_COMMAND_UUID.lower()].permissions == perms.writeable


def test_telemetry_is_notified_and_readable(bridge):
    from std_msgs.msg import Float32
    node, srv, mod = bridge()
    next(x for x in node.subscriptions_ if x.topic == '/battery/voltage').callback(Float32(data=12.345))
    node.tick()
    wait(0.1)
    assert srv.notified[-1] == (mod.SERVICE_UUID, mod.CHAR_TELEMETRY_UUID, b'{"bat":12.35,"status":"ok"}')
    assert bytes(_on_loop(node, srv.read_request, mod.CHAR_TELEMETRY_UUID)) == b'{"bat":12.35,"status":"ok"}'


def test_disconnect_stops_robot(bridge):
    node, srv, mod = bridge()
    srv.client_connected = True
    wait(0.7)
    _on_loop(node, srv.write_request, mod.CHAR_COMMAND_UUID, bytearray(b'{"lx": 0.5}'))
    srv.client_connected = False
    wait(0.7)
    last = node.cmd_vel_pub.messages[-1]
    assert (last.linear.x, last.angular.z) == (0.0, 0.0)


def test_silence_stops_robot(bridge):
    node, srv, mod = bridge(cmd_timeout=0.2)
    _on_loop(node, srv.write_request, mod.CHAR_COMMAND_UUID, bytearray(b'{"lx": 0.5}'))
    node.tick()
    assert node.cmd_vel_pub.messages[-1].linear.x == 0.5
    wait(0.25)
    node.tick()
    assert node.cmd_vel_pub.messages[-1].linear.x == 0.0


def test_shutdown_stops_robot_and_server(bridge):
    node, srv, mod = bridge()
    _on_loop(node, srv.write_request, mod.CHAR_COMMAND_UUID, bytearray(b'{"lx": 0.5}'))
    node.destroy_node()
    assert node.cmd_vel_pub.messages[-1].linear.x == 0.0
    assert srv.stopped
