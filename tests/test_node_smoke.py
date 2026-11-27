"""Smoke test: every node starts and runs.

Covers the console_scripts entry points in src/*/setup.py and the script nodes in
projects/*/nodes/ (started with python3 by the projects' launch files).

For each node module this runs its ``main()`` against the rclpy stub (tests/ros_stub)
with mock parameters set (mock_hardware, use_mock, simulate). rclpy.spin() runs a few
rounds of timer callbacks and returns, so main() goes through start-up, the timer
loop and shutdown, as it would on a robot when you press Ctrl+C.

A node fails if
  * main() raises, or hangs for more than START_TIMEOUT seconds, or
  * it logs an error while handling a programming error (AttributeError, TypeError,
    NameError, a wrong message field type, a parameter type mismatch, ...).

Missing third-party libraries are not failures: such nodes are skipped with the name of
the library (install it in CI to cover the node). Errors from the environment (no
serial port, no Bluetooth adapter, OSError) are not failures either.

Nodes with a mock mode and a timer must also publish at least one message.

Known, not yet fixed problems are listed in KNOWN_ISSUES (strict xfail: when a node
gets fixed, the test reports XPASS and the entry must be removed).
"""
import glob
import importlib
import os
import re
import sys
import threading
import traceback
import types

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, 'src')

STUBBED_ROOTS = {m.split('.')[0] for m in sys.modules if m.split('.')[0] in (
    'rclpy', 'std_msgs', 'sensor_msgs', 'geometry_msgs', 'nav_msgs', 'std_srvs', 'diagnostic_msgs',
    'visualization_msgs', 'trajectory_msgs', 'rcl_interfaces', 'builtin_interfaces',
    'example_interfaces', 'ament_index_python', 'cv_bridge')}
MOCK_OVERRIDES = {'mock_hardware': True, 'use_mock': True, 'simulate': True,
                  # dummy credentials: LLM clients only need a key to be constructed (no network
                  # at start-up); telegram stays empty so the bot does not try to connect
                  'api_key': 'smoke-test-key', 'llm_api_key': 'smoke-test-key'}
DUMMY_ENV = {'GEMINI_API_KEY': 'smoke-test-key', 'OPENROUTER_API_KEY': 'smoke-test-key'}
START_TIMEOUT = 20.0

# Exceptions that point at a bug in the node (or misuse of the ROS API), not at missing hardware.
PROGRAMMING_ERRORS = (AttributeError, NameError, TypeError, KeyError, IndexError,
                      ZeroDivisionError, AssertionError, NotImplementedError, ImportError,
                      ValueError, RecursionError)

# module -> reason. Keep short; link the fix when there is one.
KNOWN_ISSUES = {
    'xpi_comms.ble_bridge_node':
        "uses BlessServer.connected / write_request, which bless does not have (is_connected() is a "
        'coroutine, notifications go through update_value()); needs testing on BLE hardware',
    'xpi_inputs.ppm_receiver_node':
        'imports gpiozero.PulseInput, which does not exist; PPM decoding needs pigpio callbacks',
}

# Nodes that have a mock mode and a timer but publish only on events/commands.
EVENT_DRIVEN = {
    'xpi_comms.hc12_node': 'publishes only data received over the radio; use_mock mocks only the SET pin',
}


def _entry_points():
    eps = {}
    for setup in sorted(glob.glob(os.path.join(SRC, '*', 'setup.py'))):
        pkg = os.path.basename(os.path.dirname(setup))
        text = open(setup).read()
        for name, module, func in re.findall(r"['\"]\s*([\w\-]+)\s*=\s*([\w\.]+):(\w+)", text):
            eps.setdefault((module, func), []).append(f'{pkg}/{name}')
    return sorted(eps.items())


def _project_nodes():
    """projects/*/nodes/*_node.py: started as scripts by the projects' launch files."""
    eps = []
    for path in sorted(glob.glob(os.path.join(REPO, 'projects', '*', 'nodes', '*_node.py'))):
        if re.search(r'^def main\(', open(path).read(), re.M):
            rel = os.path.relpath(path, REPO)
            eps.append(((os.path.basename(path)[:-3], 'main'), [rel]))
    return eps


ENTRY_POINTS = _entry_points() + _project_nodes()


def _in_repo(filename):
    return os.path.abspath(filename).startswith(REPO + os.sep)


def _origin(exc):
    """'repo' if the innermost frame of ``exc`` is our code (or the ROS stub), else 'external'."""
    tb = traceback.extract_tb(exc.__traceback__)
    if not tb:
        return 'repo'
    return 'repo' if _in_repo(tb[-1].filename) else 'external'


def _missing_dependency(exc):
    """The third-party module behind ``exc`` (raised directly or re-raised by our code), if any."""
    seen = set()
    while exc is not None and id(exc) not in seen:
        seen.add(id(exc))
        if isinstance(exc, ModuleNotFoundError):
            root = (exc.name or '').split('.')[0]
            if root and not root.startswith('xpi') and root not in STUBBED_ROOTS:
                return exc.name
        exc = exc.__cause__ or exc.__context__
    return None


def _third_party_api(exc):
    """AttributeError on a third-party module (version/platform specific API, e.g. Blinka's board)."""
    obj = getattr(exc, 'obj', None)
    if isinstance(exc, AttributeError) and isinstance(obj, types.ModuleType):
        root = obj.__name__.split('.')[0]
        return not root.startswith('xpi') and root not in STUBBED_ROOTS
    return False


def _is_bug(exc):
    if _missing_dependency(exc) or _third_party_api(exc):
        return False
    if isinstance(exc, ModuleNotFoundError):
        # a missing xpi module, or a ROS module that does not exist upstream, is a bug;
        # a missing third-party library is an environment problem
        root = (exc.name or '').split('.')[0]
        return root.startswith('xpi') or root in STUBBED_ROOTS
    if isinstance(exc, ImportError):
        # cannot import name X from a module that exists: wrong API (e.g. rclpy.parameter)
        return True
    if isinstance(exc, OSError):
        return False
    if isinstance(exc, PROGRAMMING_ERRORS):
        # A library refusing to run off-target (Blinka: NotImplementedError, RuntimeError...) is
        # environment; the same error raised by our own code is a bug.
        return _origin(exc) == 'repo' or isinstance(exc, (AttributeError, NameError, TypeError,
                                                           AssertionError, KeyError))
    if isinstance(exc, RuntimeError):
        return _origin(exc) == 'repo'
    return _origin(exc) == 'repo'


def _describe(exc):
    tb = traceback.extract_tb(exc.__traceback__)
    where = ''
    for fr in reversed(tb):
        if _in_repo(fr.filename) and '/tests/' not in fr.filename:
            where = f' at {os.path.relpath(fr.filename, REPO)}:{fr.lineno}'
            break
    return f'{type(exc).__name__}: {exc}{where}'


def _run_main(func):
    """Run ``func`` in a thread; return (exception or None, hung?)."""
    box = {}

    def target():
        try:
            func()
        except SystemExit as e:  # some mains call sys.exit()
            if e.code not in (0, None):
                box['exc'] = e
        except KeyboardInterrupt:
            pass
        except BaseException as e:  # noqa: BLE001 - reported by the test
            box['exc'] = e

    t = threading.Thread(target=target, daemon=True)
    t.start()
    t.join(START_TIMEOUT)
    return box.get('exc'), t.is_alive()


@pytest.fixture(autouse=True)
def _dummy_credentials(monkeypatch):
    for k, v in DUMMY_ENV.items():
        monkeypatch.setenv(k, v)


def _start(ros, module_name, func_name):
    """Import and run main(). Returns the result dict or calls pytest.skip()."""
    ros.OVERRIDES.update(MOCK_OVERRIDES)
    sys.modules.pop(module_name, None)
    try:
        module = importlib.import_module(module_name)
    except BaseException as e:  # noqa: BLE001
        if _is_bug(e):
            pytest.fail(f'import failed: {_describe(e)}', pytrace=False)
        missing = _missing_dependency(e) or getattr(e, 'name', None) or type(e).__name__
        pytest.skip(f'dependency unavailable: {missing} ({type(e).__name__}: {str(e)[:80]})')
    exc, hung = _run_main(getattr(module, func_name))
    nodes = list(ros.NODES)
    bugs = [r for r in ros.LOG if r['level'] in ('error', 'fatal', 'warn')
            and r['exc'] is not None and _is_bug(r['exc'])]
    return {'exc': exc, 'hung': hung, 'nodes': nodes, 'logged_bugs': bugs}


def _params(entries):
    out = []
    for (module, func), names in entries:
        marks = []
        if module in KNOWN_ISSUES:
            marks.append(pytest.mark.xfail(reason=KNOWN_ISSUES[module], strict=True))
        out.append(pytest.param(module, func, id=module.split('.')[-1], marks=marks))
    return out


@pytest.mark.parametrize('module_name, func_name', _params(ENTRY_POINTS))
def test_node_starts(ros, module_name, func_name):
    r = _start(ros, module_name, func_name)
    problems = []
    if r['hung']:
        problems.append(f'main() did not return within {START_TIMEOUT:.0f} s')
    for rec in r['logged_bugs']:
        problems.append(f'logged "{rec["msg"][:100]}" while handling {_describe(rec["exc"])}')
    skip = None
    if r['exc'] is not None:
        if _is_bug(r['exc']):
            problems.append(f'main() raised {_describe(r["exc"])}')
        elif _missing_dependency(r['exc']):
            skip = f'dependency unavailable: {_missing_dependency(r["exc"])}'
        else:
            skip = f'needs hardware/environment: {_describe(r["exc"])}'
    elif not r['nodes'] and not r['hung']:
        problems.append('main() returned without creating a node')
    if skip and not problems:
        pytest.skip(skip)
    assert not problems, '\n'.join(dict.fromkeys(problems))


def _mock_publishers():
    out = []
    for (module, func), names in ENTRY_POINTS:
        if '.' in module:
            path = os.path.join(SRC, module.split('.')[0], *module.split('.')) + '.py'
        else:
            path = os.path.join(REPO, names[0])
        text = open(path).read()
        if re.search(r"declare_parameter\(\s*['\"](mock_hardware|use_mock|simulate)['\"]", text) \
                and 'create_timer' in text and 'create_publisher' in text:
            out.append(((module, func), names))
    return out


@pytest.mark.parametrize('module_name, func_name', _params(_mock_publishers()))
def test_mock_node_publishes(ros, module_name, func_name):
    if module_name in EVENT_DRIVEN:
        pytest.skip(f'event-driven: {EVENT_DRIVEN[module_name]}')
    r = _start(ros, module_name, func_name)
    if r['exc'] is not None or r['hung'] or not r['nodes']:
        pytest.skip('does not start (see test_node_starts)')
    published = sum(n.published() for n in r['nodes'])
    assert published > 0, (
        'mock mode is on and timers ran, but nothing was published; errors logged: '
        + '; '.join(rec['msg'][:80] for rec in ros.LOG if rec['level'] == 'error')[:400])
