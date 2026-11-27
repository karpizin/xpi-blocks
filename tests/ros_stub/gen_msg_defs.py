#!/usr/bin/env python3
"""Regenerate msg_defs.json from ROS 2 interface sources.

    git clone --depth 1 -b humble https://github.com/ros2/common_interfaces
    git clone --depth 1 -b humble https://github.com/ros2/rcl_interfaces
    git clone --depth 1 -b humble https://github.com/ros2/example_interfaces
    python3 tests/ros_stub/gen_msg_defs.py common_interfaces rcl_interfaces example_interfaces

The JSON keeps only field names, types and constants (the interface definitions
are Apache-2.0, see the upstream repositories).
"""
import json
import os
import re
import sys

PACKAGES = [
    'builtin_interfaces', 'std_msgs', 'std_srvs', 'geometry_msgs', 'sensor_msgs',
    'nav_msgs', 'diagnostic_msgs', 'visualization_msgs', 'trajectory_msgs',
    'rcl_interfaces', 'example_interfaces', 'shape_msgs', 'action_msgs', 'unique_identifier_msgs',
]
LINE = re.compile(r'^\s*([\w/]+(?:<=\d+)?(?:\[[^\]]*\])?)\s+(\w+)\s*(?:=\s*(.+?)|\s+(.+?))?\s*$')


def strip_comment(line):
    """Drop a trailing # comment, keeping '#' inside quoted string values."""
    quote = None
    for i, ch in enumerate(line):
        if quote:
            if ch == quote:
                quote = None
        elif ch in '\'"':
            quote = ch
        elif ch == '#':
            return line[:i].rstrip()
    return line.rstrip()


def parse(text):
    fields, constants = [], []
    for raw in text.splitlines():
        line = strip_comment(raw)
        if not line.strip():
            continue
        m = LINE.match(line)
        if not m:
            raise ValueError(f'cannot parse: {raw!r}')
        typ, name, const, default = m.groups()
        if const is not None:
            constants.append([typ, name, const.strip()])
        else:
            fields.append([typ, name, default.strip() if default else None])
    return {'fields': fields, 'constants': constants}


def main(roots):
    out = {}
    for root in roots:
        for dirpath, _, files in os.walk(root):
            pkg = os.path.basename(dirpath.rstrip('/'))
            kind = None
            if pkg in ('msg', 'srv'):
                kind, pkg = pkg, os.path.basename(os.path.dirname(dirpath))
            if kind is None or pkg not in PACKAGES:
                continue
            for f in sorted(files):
                name, ext = os.path.splitext(f)
                if ext != '.' + kind:
                    continue
                text = open(os.path.join(dirpath, f)).read()
                if kind == 'msg':
                    out.setdefault(pkg, {}).setdefault('msg', {})[name] = parse(text)
                else:
                    req, resp = re.split(r'^---\s*$', text, maxsplit=1, flags=re.M)
                    out.setdefault(pkg, {}).setdefault('srv', {})[name] = {
                        'Request': parse(req), 'Response': parse(resp)}
    json.dump(out, sys.stdout, indent=0, sort_keys=True)


if __name__ == '__main__':
    main(sys.argv[1:])
