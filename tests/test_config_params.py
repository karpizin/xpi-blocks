"""Every shipped parameter file (src/*/config/*.yaml) is accepted by its node.

rclpy parameters are statically typed: the type comes from the default passed to
declare_parameter(). A YAML value of another type, e.g. ``allowed_users: [123]`` for a
parameter declared with ``[]`` (which rclpy types as BYTE_ARRAY), makes the node fail at
start-up with InvalidParameterTypeException. This test loads each file like
``--params-file`` does and constructs the node it names.
"""
import glob
import importlib
import os
import re

import pytest
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, 'src')

MOCK = {'mock_hardware': True, 'use_mock': True, 'simulate': True}


def _node_classes():
    """node name passed to super().__init__ -> (module, class name)."""
    out = {}
    for path in glob.glob(os.path.join(SRC, '*', '*', '*.py')):
        text = open(path).read()
        for cls, node in re.findall(
                r"class (\w+)\(Node\):.*?super\(\).__init__\(\s*['\"]([\w]+)['\"]", text, re.S):
            rel = os.path.relpath(path, SRC).split(os.sep)
            out.setdefault(node, ('.'.join(rel[1:])[:-3], cls))
    return out


def _param_files():
    nodes = _node_classes()
    cases = []
    for path in sorted(glob.glob(os.path.join(SRC, '*', 'config', '*.yaml'))):
        data = yaml.safe_load(open(path)) or {}
        if not isinstance(data, dict):
            continue
        for node_name, section in data.items():
            if not isinstance(section, dict) or 'ros__parameters' not in section:
                continue
            name = node_name.strip('/').split('/')[-1]
            rel = os.path.relpath(path, SRC)
            # launch files often rename '<x>_node' to '<x>' (Node(name=...)); accept both
            target = nodes.get(name) or nodes.get(name + '_node')
            if target:
                cases.append(pytest.param(target, section['ros__parameters'], id=f'{rel}:{name}'))
    return cases


def _flatten(params, prefix=''):
    flat = {}
    for k, v in params.items():
        key = f'{prefix}{k}'
        if isinstance(v, dict):
            flat.update(_flatten(v, key + '.'))
        else:
            flat[key] = v
    return flat


KNOWN_ISSUES = {
}


@pytest.mark.parametrize('target, params', _param_files())
def test_param_file_is_accepted(ros, request, target, params):
    module_name, cls_name = target
    if request.node.callspec.id in KNOWN_ISSUES:
        request.applymarker(pytest.mark.xfail(reason=KNOWN_ISSUES[request.node.callspec.id], strict=True))
    try:
        module = importlib.import_module(module_name)
    except ModuleNotFoundError as e:
        pytest.skip(f'dependency unavailable: {e.name}')
    ros.OVERRIDES.update(MOCK)
    ros.OVERRIDES.update(_flatten(params))
    rclpy_errors = (ros.InvalidParameterTypeException, ros.ParameterUninitializedException,
                    ros.InvalidParameterValueException)
    try:
        getattr(module, cls_name)()
    except rclpy_errors as e:
        pytest.fail(f'{type(e).__name__}: {e}', pytrace=False)
    except OSError as e:
        pytest.skip(f'needs hardware/environment: {e}')
    for rec in ros.LOG:
        if isinstance(rec['exc'], rclpy_errors):
            pytest.fail(f'{type(rec["exc"]).__name__}: {rec["exc"]}', pytrace=False)
