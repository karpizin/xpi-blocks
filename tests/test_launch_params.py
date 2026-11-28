"""Every parameter a launch file passes to a node must be one the node declares.

ROS 2 silently ignores parameters a node never declares, and the node runs on its
defaults: smart_motor.launch.py once set the TB6612 pins as pin_ain1/pin_pwma, which the
driver does not know, so it drove its default GPIOs instead of the wired ones.
The check is static (AST of the launch files, declare_parameter calls in the node), so it
needs neither ROS nor launch_ros.
"""
import ast
import glob
import os
import re

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, 'src')

ENTRY_RE = re.compile(r"['\"]\s*([\w\-]+)\s*=\s*([\w\.]+):(\w+)")


def _entry_points():
    """(package, executable) -> path of the module that implements it."""
    eps = {}
    for setup in glob.glob(os.path.join(SRC, '*', 'setup.py')):
        pkg = os.path.basename(os.path.dirname(setup))
        for exe, module, _func in ENTRY_RE.findall(open(setup).read()):
            eps[(pkg, exe)] = os.path.join(SRC, pkg, *module.split('.')) + '.py'
    return eps


ENTRY_POINTS = _entry_points()


def _declared(module_path):
    source = open(module_path).read()
    names = set(re.findall(r"declare_parameter\(\s*['\"]([\w\.]+)['\"]", source))
    if 'declare_llm_parameters' in source:
        from xpi_llm.llm_params import LLM_PARAMETERS
        names |= set(LLM_PARAMETERS)
    return names


def _const(node):
    return node.value if isinstance(node, ast.Constant) else None


def _launch_nodes():
    """(launch file, package, executable, {param: line}) for each Node(...) with a literal dict."""
    found = []
    for path in sorted(glob.glob(os.path.join(SRC, '*', 'launch', '*.py'))):
        tree = ast.parse(open(path).read(), path)
        for call in ast.walk(tree):
            if not (isinstance(call, ast.Call) and getattr(call.func, 'id', getattr(call.func, 'attr', None)) == 'Node'):
                continue
            kw = {k.arg: k.value for k in call.keywords}
            pkg, exe = _const(kw.get('package')), _const(kw.get('executable'))
            if not (isinstance(pkg, str) and isinstance(exe, str)):
                continue
            params = {}
            plist = kw.get('parameters')
            for item in getattr(plist, 'elts', []):
                if isinstance(item, ast.Dict):
                    for key in item.keys:
                        if isinstance(_const(key), str):
                            params[key.value] = key.lineno
            found.append((os.path.relpath(path, REPO), pkg, exe, params))
    return found


LAUNCH_NODES = _launch_nodes()


def test_launch_files_found():
    assert len(LAUNCH_NODES) > 80


@pytest.mark.parametrize('launch_file,pkg,exe,params', LAUNCH_NODES,
                         ids=[f'{os.path.basename(f)}:{e}' for f, _, e, _ in LAUNCH_NODES])
def test_launch_parameters_are_declared(launch_file, pkg, exe, params):
    if not pkg.startswith('xpi_'):
        pytest.skip(f'{pkg} is not part of this repository')
    module = ENTRY_POINTS.get((pkg, exe))
    assert module, f'{launch_file}: executable {pkg}/{exe} is not in setup.py entry_points'
    unknown = {name: line for name, line in params.items() if name not in _declared(module)}
    assert not unknown, (
        f'{launch_file}: {exe} does not declare ' +
        ', '.join(f'{n!r} (line {line})' for n, line in sorted(unknown.items())) +
        ' — ROS 2 would ignore them and use the defaults')
