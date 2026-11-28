"""Package metadata: every node module has a ros2 run entry point, manifests are filled in."""
import ast
import pathlib
import re
import xml.etree.ElementTree as ET

import pytest

SRC = pathlib.Path(__file__).resolve().parent.parent / 'src'
PACKAGES = sorted(p for p in SRC.iterdir() if (p / 'package.xml').exists())


def _entry_points(pkg):
    tree = ast.parse((pkg / 'setup.py').read_text())
    targets = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            m = re.fullmatch(r'\s*[\w.-]+\s*=\s*([\w.]+):\w+\s*', node.value)
            if m:
                targets.add(m.group(1))
    return targets


@pytest.mark.parametrize('pkg', PACKAGES, ids=lambda p: p.name)
def test_every_node_module_has_entry_point(pkg):
    registered = _entry_points(pkg)
    missing = []
    for f in sorted((pkg / pkg.name).rglob('*_node.py')):
        if 'def main(' not in f.read_text():
            continue
        module = '.'.join(f.relative_to(pkg).with_suffix('').parts)
        if module not in registered:
            missing.append(module)
    assert not missing, f'not runnable with ros2 run: {missing}'


@pytest.mark.parametrize('pkg', PACKAGES, ids=lambda p: p.name)
def test_manifest_filled_in(pkg):
    root = ET.parse(pkg / 'package.xml').getroot()
    emails = [m.get('email') for m in root.findall('maintainer')]
    assert emails and not any('example.com' in e for e in emails)
    deps = [d for tag in ('depend', 'exec_depend', 'build_depend') for d in root.findall(tag)]
    assert deps, 'package.xml declares no dependencies'
    assert 'example.com' not in (pkg / 'setup.py').read_text()
