"""ROS 2 message/service classes built from msg_defs.json.

Mirrors the checks of rosidl_generator_py (Humble) so that the same mistakes
fail here as on a robot:

* unknown field -> AttributeError (__slots__); unknown constructor kwarg -> AssertionError
* wrong type -> AssertionError, e.g. Float32(data=5) (int for a float field) or a
  numpy.float32 for a float64 field
* integer out of range, float outside float32/float64 range (inf is allowed)
* numeric arrays are stored as numpy arrays (fixed size) / array.array (sequences)
"""
import array
import json
import math
import os
import re
import sys
import types
from collections import UserList, UserString
from collections.abc import Sequence, Set

import numpy

_DEFS = json.load(open(os.path.join(os.path.dirname(__file__), 'msg_defs.json')))

INT_BITS = {'int8': 8, 'int16': 16, 'int32': 32, 'int64': 64}
UINT_BITS = {'uint8': 8, 'uint16': 16, 'uint32': 32, 'uint64': 64, 'byte': 8, 'octet': 8}
FLOAT_BOUND = {'float32': 3.402823466e+38, 'float64': 1.7976931348623157e+308}
# rosidl: these element types are stored as numpy (fixed) / array.array (sequence)
SPECIAL = {
    'float32': ('f', numpy.float32), 'float64': ('d', numpy.float64),
    'int8': ('b', numpy.int8), 'uint8': ('B', numpy.uint8),
    'int16': ('h', numpy.int16), 'uint16': ('H', numpy.uint16),
    'int32': ('i', numpy.int32), 'uint32': ('I', numpy.uint32),
    'int64': ('q', numpy.int64), 'uint64': ('Q', numpy.uint64),
}
TYPE_RE = re.compile(r'^(?P<base>[\w/]+?)(?:<=(?P<strmax>\d+))?(?:\[(?P<arr>[^\]]*)\])?$')

_classes = {}   # 'pkg/msg/Name' -> class


class FieldType:
    def __init__(self, spec, pkg):
        m = TYPE_RE.match(spec)
        if not m:
            raise ValueError(spec)
        base = m.group('base')
        self.str_max = int(m.group('strmax')) if m.group('strmax') else None
        arr = m.group('arr')
        self.is_array = arr is not None
        self.fixed = int(arr) if arr and arr.isdigit() else None
        self.bound = int(arr[2:]) if arr and arr.startswith('<=') else None
        if base == 'Header':
            base = 'std_msgs/Header'
        if base == 'float':
            base = 'float32'
        if base == 'double':
            base = 'float64'
        self.base = base
        self.primitive = base in INT_BITS or base in UINT_BITS or base in FLOAT_BOUND or \
            base in ('bool', 'string', 'wstring', 'char')
        if not self.primitive:
            self.nested = base if '/' in base else f'{pkg}/{base}'

    # ---------------------------------------------------------------- values
    def python_type(self):
        b = self.base
        if b == 'bool':
            return bool
        if b in ('byte', 'octet'):
            return bytes
        if b in INT_BITS or b in UINT_BITS:
            return int
        if b in FLOAT_BOUND:
            return float
        if b in ('string', 'wstring', 'char'):
            return str
        return msg_class(self.nested)

    def scalar_default(self):
        if not self.primitive:
            return msg_class(self.nested)()
        return {bool: False, int: 0, float: 0.0, str: '', bytes: b'\x00'}[self.python_type()]

    def default(self, literal=None):
        if literal is not None:
            return self.coerce(parse_literal(literal, self))
        if not self.is_array:
            return self.scalar_default()
        if self.base in SPECIAL:
            tc, dt = SPECIAL[self.base]
            if self.fixed is not None:
                return numpy.zeros(self.fixed, dtype=dt)
            return array.array(tc)
        if self.fixed is not None:
            return [self.scalar_default() for _ in range(self.fixed)]
        return []

    def check_scalar(self, name, v):
        b = self.base
        t = self.python_type()
        if b in ('byte', 'octet'):
            assert isinstance(v, bytes) and len(v) == 1, \
                f"The '{name}' field must be of type 'bytes' or 'ByteString' with length 1"
            return
        if not self.primitive:
            assert isinstance(v, t), f"The '{name}' field must be a sub message of type '{t.__name__}'"
            return
        if b in ('string', 'wstring', 'char'):
            if self.str_max is not None:
                assert isinstance(v, (str, UserString)) and len(v) <= self.str_max, \
                    f"The '{name}' field must be string value not longer than {self.str_max}"
            else:
                assert isinstance(v, str), f"The '{name}' field must be of type 'str'"
            return
        assert isinstance(v, t), f"The '{name}' field must be of type '{t.__name__}'"
        if b in INT_BITS:
            bound = 2 ** (INT_BITS[b] - 1)
            assert -bound <= v < bound, f"The '{name}' field must be an integer in [{-bound}, {bound - 1}]"
        elif b in UINT_BITS:
            bound = 2 ** UINT_BITS[b]
            assert 0 <= v < bound, f"The '{name}' field must be an unsigned integer in [0, {bound - 1}]"
        elif b in FLOAT_BOUND:
            bound = FLOAT_BOUND[b]
            assert not (v < -bound or v > bound) or math.isinf(v), \
                f"The '{name}' field must be a {b} in [{-bound}, {bound}]"

    def coerce(self, value, name='value'):
        """Validate like the generated setter and return the stored value."""
        if not self.is_array:
            self.check_scalar(name, value)
            return value
        if self.base in SPECIAL:
            tc, dt = SPECIAL[self.base]
            if self.fixed is not None and isinstance(value, numpy.ndarray):
                assert value.dtype == dt, f"The '{name}' numpy.ndarray() must have the dtype of '{dt.__name__}'"
                assert value.size == self.fixed, f"The '{name}' numpy.ndarray() must have a size of {self.fixed}"
                return value
            if self.fixed is None and isinstance(value, array.array):
                assert value.typecode == tc, f"The '{name}' array.array() must have the type code of '{tc}'"
                if self.bound is not None:
                    assert len(value) <= self.bound
                return value
        assert ((isinstance(value, (Sequence, Set, UserList))) and
                not isinstance(value, (str, UserString))), \
            f"The '{name}' field must be a set or sequence"
        value = list(value)
        if self.fixed is not None:
            assert len(value) == self.fixed, f"The '{name}' field must be a set or sequence with length {self.fixed}"
        if self.bound is not None:
            assert len(value) <= self.bound, f"The '{name}' field must be a set or sequence with length <= {self.bound}"
        for v in value:
            self.check_scalar(name, v)
        if self.base in SPECIAL:
            tc, dt = SPECIAL[self.base]
            return numpy.array(value, dtype=dt) if self.fixed is not None else array.array(tc, value)
        return value


def parse_literal(text, ftype):
    text = text.strip()
    t = ftype.python_type() if ftype.primitive else None
    if ftype.is_array:
        inner = text.strip('[]')
        items = [x.strip() for x in inner.split(',') if x.strip()]
        scalar = FieldType(ftype.base, '')
        return [parse_literal(x, scalar) for x in items]
    if t is bool:
        return text.lower() in ('true', '1')
    if t is int:
        return int(text, 0)
    if t is float:
        return float(text)
    if t is str:
        return text[1:-1] if text[:1] in '"\'' else text
    return text


def _make_class(full_name, spec):
    pkg, kind, name = full_name.split('/')
    fields = [(n, FieldType(t, pkg), d) for t, n, d in spec['fields']]
    field_names = tuple(f for f, _, _ in fields)

    def __init__(self, **kwargs):
        bad = sorted(k for k in kwargs if k not in field_names)
        assert not bad, 'Invalid arguments passed to constructor: %s' % ', '.join(bad)
        for fname, ftype, lit in fields:
            if fname in kwargs:
                setattr(self, fname, kwargs[fname])
            else:
                object.__setattr__(self, '_v_' + fname, ftype.default(lit))

    def __repr__(self):
        args = ', '.join(f'{f}={getattr(self, f)!r}' for f in field_names)
        return f'{pkg}.{kind}.{name}({args})'

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        for f in field_names:
            a, b = getattr(self, f), getattr(other, f)
            if isinstance(a, numpy.ndarray) or isinstance(b, numpy.ndarray):
                if not numpy.array_equal(a, b):
                    return False
            elif a != b:
                return False
        return True

    def make_prop(fname, ftype):
        def get(self):
            return object.__getattribute__(self, '_v_' + fname)

        def set_(self, value):
            object.__setattr__(self, '_v_' + fname, ftype.coerce(value, fname))
        return property(get, set_)

    ns = {'__init__': __init__, '__repr__': __repr__, '__eq__': __eq__, '__hash__': None,
          '__slots__': tuple('_v_' + f for f in field_names),
          '__module__': f'{pkg}.{kind}',
          '_fields_and_field_types': {f: t.base for f, t, _ in fields}}
    for fname, ftype, _ in fields:
        ns[fname] = make_prop(fname, ftype)
    for ctype, cname, cval in spec['constants']:
        ns[cname] = parse_literal(cval, FieldType(ctype, pkg))
    return type(name, (), ns)


def msg_class(full_name):
    """'std_msgs/Header' or 'std_msgs/msg/Header' -> class."""
    parts = full_name.split('/')
    if len(parts) == 2:
        full_name = f'{parts[0]}/msg/{parts[1]}'
    if full_name not in _classes:
        pkg, kind, name = full_name.split('/')
        try:
            spec = _DEFS[pkg][kind][name]
        except KeyError:
            raise ImportError(f"cannot import name '{name}' from '{pkg}.{kind}'") from None
        _classes[full_name] = _make_class(full_name, spec)
    return _classes[full_name]


def _srv_class(pkg, name):
    key = f'{pkg}/srv/{name}'
    if key not in _classes:
        spec = _DEFS[pkg]['srv'][name]
        req = _make_class(f'{pkg}/srv/{name}_Request', spec['Request'])
        resp = _make_class(f'{pkg}/srv/{name}_Response', spec['Response'])
        _classes[key] = type(name, (), {'Request': req, 'Response': resp, '__module__': f'{pkg}.srv'})
    return _classes[key]


class _InterfaceModule(types.ModuleType):
    """pkg.msg / pkg.srv module: only names that exist upstream can be imported."""

    def __init__(self, pkg, kind):
        super().__init__(f'{pkg}.{kind}')
        self._pkg, self._kind = pkg, kind
        self.__all__ = sorted(_DEFS[pkg].get(kind, {}))

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        if name not in _DEFS[self._pkg].get(self._kind, {}):
            raise AttributeError(f"module '{self.__name__}' has no attribute '{name}'")
        cls = msg_class(f'{self._pkg}/msg/{name}') if self._kind == 'msg' else _srv_class(self._pkg, name)
        setattr(self, name, cls)
        return cls


def install():
    for pkg, kinds in _DEFS.items():
        top = types.ModuleType(pkg)
        top.__path__ = []
        sys.modules[pkg] = top
        for kind in kinds:
            mod = _InterfaceModule(pkg, kind)
            sys.modules[f'{pkg}.{kind}'] = mod
            setattr(top, kind, mod)
