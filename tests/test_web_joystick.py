"""Web joystick: who may connect (token, Origin, bind address) and what gets through."""
import pytest

pytest.importorskip('fastapi')
pytest.importorskip('httpx')
from starlette.testclient import TestClient  # noqa: E402
from starlette.websockets import WebSocketDisconnect  # noqa: E402

TOKEN = 'test-token-123'


@pytest.fixture
def joystick(ros, monkeypatch):
    import xpi_inputs.web_joystick_node as mod
    runs = []
    monkeypatch.setattr(mod.WebJoystickNode, 'run_server',
                        lambda self: runs.append((self.host, self.port)))
    monkeypatch.delenv(mod.TOKEN_ENV, raising=False)

    def make(**params):
        ros.OVERRIDES.update(params)
        node = mod.WebJoystickNode()
        node.server_thread.join(1)
        cmd_vel = next(p for p in node.publishers_ if p.topic == 'cmd_vel')
        return node, TestClient(node.app), cmd_vel, runs
    return make


def _drive(client, url, headers=None):
    with client.websocket_connect(url, headers=headers or {}) as ws:
        ws.send_text('{"lx": 0.5, "az": 0.0}')
        ws.send_text('{"lx": 0.5, "az": 0.0}')   # second send: the first was processed


def test_binds_to_loopback_by_default(joystick):
    node, client, cmd_vel, runs = joystick(token=TOKEN)
    assert runs == [('127.0.0.1', 8080)]


def test_host_parameter(joystick):
    node, client, cmd_vel, runs = joystick(token=TOKEN, host='0.0.0.0')
    assert runs == [('0.0.0.0', 8080)]
    from ros_stub import rclpy_stub
    assert any('trusted network' in r['msg'] for r in rclpy_stub.LOG if r['level'] == 'warn')


@pytest.mark.parametrize('url', ['/ws', '/ws?token=', '/ws?token=wrong'])
def test_rejects_missing_or_wrong_token(joystick, url):
    node, client, cmd_vel, runs = joystick(token=TOKEN)
    with pytest.raises(WebSocketDisconnect) as e:
        _drive(client, url)
    assert e.value.code == 1008
    assert cmd_vel.messages == []


def test_accepts_correct_token_and_drives(joystick):
    node, client, cmd_vel, runs = joystick(token=TOKEN)
    _drive(client, f'/ws?token={TOKEN}')
    assert any(m.linear.x == pytest.approx(0.5) for m in cmd_vel.messages)
    assert cmd_vel.messages[-1].linear.x == 0.0      # disconnect = stop


def test_rejects_foreign_origin_even_with_token(joystick):
    """Cross-site WebSocket hijacking: a page from another site in the user's browser."""
    node, client, cmd_vel, runs = joystick(token=TOKEN)
    with pytest.raises(WebSocketDisconnect):
        _drive(client, f'/ws?token={TOKEN}', {'origin': 'http://evil.example'})
    assert cmd_vel.messages == []


def test_rejects_dns_rebinding_origin(joystick):
    node, client, cmd_vel, runs = joystick(token=TOKEN)
    with pytest.raises(WebSocketDisconnect):
        _drive(client, f'/ws?token={TOKEN}', {'origin': 'http://attacker.example:8080'})


def test_same_origin_page_is_allowed(joystick):
    node, client, cmd_vel, runs = joystick(token=TOKEN)
    _drive(client, f'/ws?token={TOKEN}', {'origin': 'http://testserver'})   # TestClient's Host
    assert cmd_vel.messages


def test_allowed_origins_parameter(joystick):
    node, client, cmd_vel, runs = joystick(
        token=TOKEN, allowed_origins='http://dash.local:3000, https://ops.example/')
    _drive(client, f'/ws?token={TOKEN}', {'origin': 'https://ops.example'})
    assert cmd_vel.messages


def test_generated_token_is_logged_with_url(joystick):
    node, client, cmd_vel, runs = joystick(token='')
    assert len(node.token) >= 16
    from ros_stub import rclpy_stub
    assert any(f'#token={node.token}' in r['msg'] for r in rclpy_stub.LOG)
    _drive(client, f'/ws?token={node.token}')
    assert cmd_vel.messages


def test_env_token_wins_and_is_not_logged(joystick, monkeypatch):
    monkeypatch.setenv('XPI_WEB_JOYSTICK_TOKEN', 'from-env')
    node, client, cmd_vel, runs = joystick(token='from-param')
    assert node.token == 'from-env'
    from ros_stub import rclpy_stub
    assert not any('from-env' in r['msg'] for r in rclpy_stub.LOG)


def test_page_and_local_nipplejs_are_served(joystick):
    node, client, cmd_vel, runs = joystick(token=TOKEN)
    page = client.get('/')
    assert page.status_code == 200
    assert 'cdnjs' not in page.text and 'src="nipplejs.min.js"' in page.text
    assert '#token' in page.text or 'location.hash' in page.text
    js = client.get('/nipplejs.min.js')
    assert js.status_code == 200 and 'nipplejs' in js.text


def test_uvicorn_gets_host_and_message_size_limit(ros, monkeypatch):
    import xpi_inputs.web_joystick_node as mod
    seen = []
    monkeypatch.setattr(mod.uvicorn, 'run', lambda app, **kw: seen.append(kw))
    ros.OVERRIDES.update({'token': TOKEN})
    node = mod.WebJoystickNode()
    node.server_thread.join(1)
    assert seen and seen[0]['host'] == '127.0.0.1'
    assert seen[0]['ws_max_size'] == mod.MAX_MESSAGE_BYTES
