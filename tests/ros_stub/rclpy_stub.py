"""In-process stand-in for rclpy (Humble), enough to construct and tick nodes without ROS.

Faithful where mistakes are common:
* only names that exist in the real rclpy modules can be imported (rclpy_api.json)
* parameters are statically typed as in rclpy: the default's type is inferred
  (an empty list becomes BYTE_ARRAY), overrides of another type raise
  InvalidParameterTypeException, a None default without dynamic typing raises
  ParameterUninitializedException on get_parameter()
* publish() of the wrong message type raises TypeError; loggers accept str only
* create_publisher/create_subscription require a QoS argument
* topic and node names are validated

Test hooks: ``OVERRIDES`` (parameter overrides applied to every node), ``NODES``
(created nodes), ``LOG`` (records with the exception being handled, if any),
``Node.tick()`` / ``Node.publishers_`` / ``Node.subscriptions_`` / ``Node.timers_``.
"""
import array
import enum
import json
import os
import re
import sys
import threading
import time as _time
import types

_sleep = _time.sleep    # the real one, even if a test caps time.sleep

_API = json.load(open(os.path.join(os.path.dirname(__file__), 'rclpy_api.json')))

OVERRIDES = {}
NODES = []
LOG = []        # dicts: level, node, msg, exc (exception instance being handled or None)
TICK_MAX_WAIT = 0.05  # s; longest real wait per tick round
SPIN_TICKS = 3  # how many rounds of timer callbacks rclpy.spin() runs before returning


class StubError(AssertionError):
    """Raised for misuse that would also fail with real rclpy."""


# ----------------------------------------------------------------------------- exceptions
class ParameterException(Exception):
    pass


class ParameterNotDeclaredException(ParameterException):
    pass


class ParameterAlreadyDeclaredException(ParameterException):
    pass


class InvalidParameterTypeException(ParameterException):
    pass


class InvalidParameterValueException(ParameterException):
    pass


class ParameterUninitializedException(ParameterException):
    pass


class InvalidTopicNameException(Exception):
    pass


class InvalidNodeNameException(Exception):
    pass


class ROSInterruptException(Exception):
    pass


# ----------------------------------------------------------------------------- parameters
class Parameter:
    class Type(enum.IntEnum):
        NOT_SET = 0
        BOOL = 1
        INTEGER = 2
        DOUBLE = 3
        STRING = 4
        BYTE_ARRAY = 5
        BOOL_ARRAY = 6
        INTEGER_ARRAY = 7
        DOUBLE_ARRAY = 8
        STRING_ARRAY = 9

        @classmethod
        def from_parameter_value(cls, v):
            if v is None:
                return cls.NOT_SET
            if isinstance(v, bool):
                return cls.BOOL
            if isinstance(v, int):
                return cls.INTEGER
            if isinstance(v, float):
                return cls.DOUBLE
            if isinstance(v, str):
                return cls.STRING
            if isinstance(v, (list, tuple, array.array)):
                if all(isinstance(x, bytes) for x in v):
                    return cls.BYTE_ARRAY       # also the empty list, as in rclpy
                if all(isinstance(x, bool) for x in v):
                    return cls.BOOL_ARRAY
                if all(isinstance(x, int) for x in v):
                    return cls.INTEGER_ARRAY
                if all(isinstance(x, float) for x in v):
                    return cls.DOUBLE_ARRAY
                if all(isinstance(x, str) for x in v):
                    return cls.STRING_ARRAY
                raise TypeError('The given value is not a list of one of the allowed types'
                                f" '{v}'.")
            raise TypeError(f"The given value is not one of the allowed types '{v}'.")

    def __init__(self, name, type_=None, value=None):
        if type_ is None:
            type_ = Parameter.Type.from_parameter_value(value)
        self._name, self._type, self._value = name, Parameter.Type(type_), value

    @property
    def name(self):
        return self._name

    @property
    def type_(self):
        return self._type

    @property
    def value(self):
        return self._value

    def get_parameter_value(self):
        from rcl_interfaces.msg import ParameterValue
        pv = ParameterValue(type=int(self._type))
        field = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value',
                 5: 'byte_array_value', 6: 'bool_array_value', 7: 'integer_array_value',
                 8: 'double_array_value', 9: 'string_array_value'}.get(int(self._type))
        if field:
            setattr(pv, field, self._value)
        return pv


def _descriptor():
    from rcl_interfaces.msg import ParameterDescriptor
    return ParameterDescriptor()


# ----------------------------------------------------------------------------- QoS
class _Policy(enum.IntEnum):
    pass


class ReliabilityPolicy(enum.IntEnum):
    SYSTEM_DEFAULT = 0
    RELIABLE = 1
    BEST_EFFORT = 2
    UNKNOWN = 3


class HistoryPolicy(enum.IntEnum):
    SYSTEM_DEFAULT = 0
    KEEP_LAST = 1
    KEEP_ALL = 2
    UNKNOWN = 3


class DurabilityPolicy(enum.IntEnum):
    SYSTEM_DEFAULT = 0
    TRANSIENT_LOCAL = 1
    VOLATILE = 2
    UNKNOWN = 3


class LivelinessPolicy(enum.IntEnum):
    SYSTEM_DEFAULT = 0
    AUTOMATIC = 1
    MANUAL_BY_TOPIC = 3
    UNKNOWN = 4


class QoSProfile:
    def __init__(self, **kwargs):
        allowed = {'history', 'depth', 'reliability', 'durability', 'lifespan', 'deadline',
                   'liveliness', 'liveliness_lease_duration', 'avoid_ros_namespace_conventions'}
        bad = set(kwargs) - allowed
        if bad:
            raise TypeError(f'Invalid arguments passed to constructor: {", ".join(sorted(bad))}')
        if 'history' not in kwargs and 'depth' not in kwargs:
            raise AssertionError('QoSProfile requires either history or depth')
        self.__dict__.update(kwargs)


qos_profile_sensor_data = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
qos_profile_system_default = QoSProfile(depth=10)
qos_profile_services_default = QoSProfile(depth=10)
qos_profile_parameters = QoSProfile(depth=1000)


def _check_qos(qos):
    if isinstance(qos, bool) or not isinstance(qos, (int, QoSProfile)):
        raise TypeError(f'Expected QoSProfile or int, but received {type(qos)!r}')


# ----------------------------------------------------------------------------- names
_TOKEN = r'[A-Za-z_][A-Za-z0-9_]*'
_TOPIC = re.compile(rf'^(~|~/|/)?({_TOKEN}|{{\w+}})(/({_TOKEN}|{{\w+}}))*$|^~$')
_NODE = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')


def _check_topic(name, what='topic'):
    if not isinstance(name, str) or not _TOPIC.match(name):
        raise InvalidTopicNameException(f'Invalid {what} name {name!r}')


# ----------------------------------------------------------------------------- time
class Duration:
    def __init__(self, *, seconds=0, nanoseconds=0):
        self.nanoseconds = int(seconds * 1e9) + int(nanoseconds)

    def to_msg(self):
        from builtin_interfaces.msg import Duration as D
        return D(sec=self.nanoseconds // 10**9, nanosec=self.nanoseconds % 10**9)


class Time:
    def __init__(self, *, seconds=0, nanoseconds=0, clock_type=None):
        self.nanoseconds = int(seconds * 1e9) + int(nanoseconds)

    def to_msg(self):
        from builtin_interfaces.msg import Time as T
        return T(sec=self.nanoseconds // 10**9, nanosec=self.nanoseconds % 10**9)

    def seconds_nanoseconds(self):
        return self.nanoseconds // 10**9, self.nanoseconds % 10**9

    def __sub__(self, other):
        if isinstance(other, Time):
            return Duration(nanoseconds=self.nanoseconds - other.nanoseconds)
        return Time(nanoseconds=self.nanoseconds - other.nanoseconds)

    def __add__(self, other):
        return Time(nanoseconds=self.nanoseconds + other.nanoseconds)

    def __lt__(self, o):
        return self.nanoseconds < o.nanoseconds

    def __le__(self, o):
        return self.nanoseconds <= o.nanoseconds

    def __gt__(self, o):
        return self.nanoseconds > o.nanoseconds

    def __ge__(self, o):
        return self.nanoseconds >= o.nanoseconds


class Clock:
    def now(self):
        return Time(nanoseconds=_time.time_ns())


# ----------------------------------------------------------------------------- logging
class Logger:
    def __init__(self, name):
        self.name = name

    def _log(self, level, message, **kwargs):
        if not isinstance(message, str):
            # rclpy passes the message to C as std::string
            raise TypeError(f'logger.{level}() expects a str, got {type(message).__name__}')
        exc = sys.exc_info()[1]
        LOG.append({'level': level, 'node': self.name, 'msg': message, 'exc': exc})
        return True

    def debug(self, m, **k):
        return self._log('debug', m, **k)

    def info(self, m, **k):
        return self._log('info', m, **k)

    def warning(self, m, **k):
        return self._log('warn', m, **k)

    warn = warning

    def error(self, m, **k):
        return self._log('error', m, **k)

    def fatal(self, m, **k):
        return self._log('fatal', m, **k)

    def get_child(self, name):
        return Logger(f'{self.name}.{name}')

    def set_level(self, level):
        pass

    def get_effective_level(self):
        return 20

    def is_enabled_for(self, severity):
        return True


def get_logger(name):
    return Logger(name)


# ----------------------------------------------------------------------------- entities
def resolve_name(node, name):
    """Expand '~/x' and relative names like rcl does (node namespace is '/' here)."""
    ns = node.get_namespace().rstrip('/')
    if name == '~' or name.startswith('~/'):
        return f'{ns}/{node.get_name()}{name[1:]}'
    if name.startswith('/'):
        return name
    return f'{ns}/{name}'


_delivery = threading.local()
MAX_DELIVERY_DEPTH = 8


class Publisher:
    def __init__(self, node, msg_type, topic, qos):
        self.node, self.msg_type, self.topic, self.qos = node, msg_type, topic, qos
        self.messages = []

    @property
    def topic_name(self):
        return self.topic

    def publish(self, msg):
        if not isinstance(msg, (self.msg_type, bytes)):
            raise TypeError(f'Expected {self.msg_type.__name__}, got {type(msg).__name__}')
        self.messages.append(msg)
        # In-process delivery to subscriptions on the same topic and type (any stub node)
        depth = getattr(_delivery, 'depth', 0)
        if depth >= MAX_DELIVERY_DEPTH:
            return
        target = resolve_name(self.node, self.topic)
        _delivery.depth = depth + 1
        try:
            for n in list(NODES):
                if n.destroyed:
                    continue
                for sub in list(n.subscriptions_):
                    if sub.msg_type is self.msg_type and resolve_name(n, sub.topic) == target:
                        sub.callback(msg)
        finally:
            _delivery.depth = depth

    def get_subscription_count(self):
        return 0

    def destroy(self):
        pass


class Subscription:
    def __init__(self, node, msg_type, topic, callback, qos):
        self.node, self.msg_type, self.topic, self.callback, self.qos = node, msg_type, topic, callback, qos

    @property
    def topic_name(self):
        return self.topic

    def destroy(self):
        pass


class Timer:
    def __init__(self, period, callback):
        if not isinstance(period, (int, float)) or isinstance(period, bool):
            raise TypeError(f'timer period must be a number, got {type(period).__name__}')
        if period <= 0:
            raise ValueError(f'timer period must be > 0, got {period}')
        self.timer_period_ns = int(period * 1e9)
        self.callback = callback
        self.canceled = False

    def cancel(self):
        self.canceled = True

    def reset(self):
        self.canceled = False

    def is_canceled(self):
        return self.canceled

    def destroy(self):
        self.canceled = True


class Service:
    def __init__(self, srv_type, name, callback):
        self.srv_type, self.srv_name, self.callback = srv_type, name, callback

    def destroy(self):
        pass


class Client:
    def __init__(self, srv_type, name):
        self.srv_type, self.srv_name = srv_type, name

    def wait_for_service(self, timeout_sec=None):
        return False

    def service_is_ready(self):
        return False

    def call_async(self, request):
        raise RuntimeError('no service server in the smoke test')

    def destroy(self):
        pass


class SetParametersResult:
    def __init__(self, successful=True, reason=''):
        self.successful, self.reason = successful, reason


# ----------------------------------------------------------------------------- Node
_NODE_METHODS = set(_API['Node methods'])


class Node:
    def __init__(self, node_name, *, context=None, cli_args=None, namespace=None,
                 use_global_arguments=True, enable_rosout=True, start_parameter_services=True,
                 parameter_overrides=None, allow_undeclared_parameters=False,
                 automatically_declare_parameters_from_overrides=False):
        if not isinstance(node_name, str) or not _NODE.match(node_name):
            raise InvalidNodeNameException(f'Invalid node name {node_name!r}')
        self._name = node_name
        self._namespace = namespace or '/'
        self._parameters = {}
        self._descriptors = {}
        self._overrides = dict(OVERRIDES)
        for p in parameter_overrides or []:
            self._overrides[p.name] = p.value
        self._allow_undeclared = allow_undeclared_parameters
        self._param_callbacks = []
        self._logger = Logger(node_name)
        self._clock = Clock()
        self.publishers_, self.subscriptions_, self.timers_ = [], [], []
        self.services_, self.clients_ = [], []
        self.destroyed = False
        NODES.append(self)

    # Read-only properties of rclpy.node.Node (assigning to them raises AttributeError there too)
    @property
    def publishers(self):
        return iter(self.publishers_)

    @property
    def subscriptions(self):
        return iter(self.subscriptions_)

    @property
    def timers(self):
        return iter(self.timers_)

    @property
    def services(self):
        return iter(self.services_)

    @property
    def clients(self):
        return iter(self.clients_)

    @property
    def guards(self):
        return iter(())

    @property
    def waitables(self):
        return iter(())

    @property
    def context(self):
        return None

    @property
    def default_callback_group(self):
        return None

    @property
    def executor(self):
        return getattr(self, '_executor', None)

    @executor.setter
    def executor(self, value):
        object.__setattr__(self, '_executor', value)

    @property
    def handle(self):
        return None

    @handle.setter
    def handle(self, value):
        raise AttributeError('handle cannot be modified after node creation')

    def __getattr__(self, name):
        # A real Node method this stub does not implement: tell the test, don't pretend
        if name in _NODE_METHODS:
            raise NotImplementedError(f'rclpy stub: Node.{name}() is not implemented')
        raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")

    # --- identity
    def get_name(self):
        return self._name

    def get_namespace(self):
        return self._namespace

    def get_fully_qualified_name(self):
        return ('' if self._namespace == '/' else self._namespace) + '/' + self._name

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    # --- parameters (rclpy semantics, static typing)
    def declare_parameter(self, name, value=None, descriptor=None, ignore_override=False):
        return self.declare_parameters('', [(name, value, descriptor or _descriptor())],
                                       ignore_override)[0]

    def declare_parameters(self, namespace, parameters, ignore_override=False):
        from rcl_interfaces.msg import ParameterDescriptor
        result = []
        for tup in parameters:
            if not 1 <= len(tup) <= 3:
                raise TypeError(f'Invalid parameter tuple length: {tup}')
            name = tup[0]
            second = tup[1] if len(tup) > 1 else None
            desc = tup[2] if len(tup) > 2 and tup[2] is not None else ParameterDescriptor()
            if not isinstance(name, str):
                raise TypeError(f'parameter name {name!r} is not a str')
            if not isinstance(desc, ParameterDescriptor):
                raise TypeError(f'descriptor for {name!r} is not a ParameterDescriptor')
            if len(tup) == 1:
                desc.dynamic_typing = True
            if isinstance(second, Parameter.Type):
                if second == Parameter.Type.NOT_SET:
                    raise ValueError(f'Cannot declare parameter {{{name}}} as statically typed of type NOT_SET')
                desc.type = int(second)
                value = None
            else:
                value = second
                if not desc.dynamic_typing and value is not None:
                    desc.type = int(Parameter.Type.from_parameter_value(value))
            full = f'{namespace}.{name}' if namespace else name
            if not ignore_override and full in self._overrides:
                value = self._overrides[full]
            if full in self._parameters:
                raise ParameterAlreadyDeclaredException([full])
            ptype = Parameter.Type.from_parameter_value(value)
            if not desc.dynamic_typing and desc.type != 0 and value is not None and int(ptype) != desc.type:
                raise InvalidParameterTypeException(
                    f"Trying to set parameter '{full}' to '{value}' of type '{ptype.name}', "
                    f"expecting type '{Parameter.Type(desc.type).name}'")
            p = Parameter(full, ptype, list(value) if isinstance(value, tuple) else value)
            for cb in self._param_callbacks:
                r = cb([p])
                if not getattr(r, 'successful', True):
                    raise InvalidParameterValueException(full, value, getattr(r, 'reason', ''))
            self._parameters[full] = p
            self._descriptors[full] = desc
            result.append(p)
        return result

    def has_parameter(self, name):
        return name in self._parameters

    def get_parameter(self, name):
        if name in self._parameters:
            p = self._parameters[name]
            if p.type_ != Parameter.Type.NOT_SET or self._descriptors[name].dynamic_typing:
                return p
            raise ParameterUninitializedException(name)
        if self._allow_undeclared:
            return Parameter(name, Parameter.Type.NOT_SET, None)
        raise ParameterNotDeclaredException(name)

    def get_parameters(self, names):
        return [self.get_parameter(n) for n in names]

    def get_parameter_or(self, name, alternative_value=None):
        if name not in self._parameters:
            return alternative_value if alternative_value is not None else Parameter(name, Parameter.Type.NOT_SET)
        return self._parameters[name]

    def get_parameters_by_prefix(self, prefix):
        pre = prefix + '.' if prefix else ''
        return {k[len(pre):]: v for k, v in self._parameters.items() if k.startswith(pre)}

    def set_parameters(self, parameter_list):
        results = []
        for p in parameter_list:
            if p.name not in self._parameters and not self._allow_undeclared:
                raise ParameterNotDeclaredException(p.name)
            d = self._descriptors.get(p.name)
            if d is not None and not d.dynamic_typing and d.type != 0 and int(p.type_) != d.type:
                results.append(SetParametersResult(False, 'Wrong parameter type'))
                continue
            ok = True
            for cb in self._param_callbacks:
                r = cb([p])
                ok = ok and getattr(r, 'successful', True)
            if ok:
                self._parameters[p.name] = p
            results.append(SetParametersResult(ok))
        return results

    def add_on_set_parameters_callback(self, callback):
        self._param_callbacks.append(callback)

    def remove_on_set_parameters_callback(self, callback):
        self._param_callbacks.remove(callback)

    # --- entities
    def create_publisher(self, msg_type, topic, qos_profile, *, callback_group=None,
                         event_callbacks=None, qos_overriding_options=None, publisher_class=None):
        _check_qos(qos_profile)
        _check_topic(topic)
        p = Publisher(self, msg_type, topic, qos_profile)
        self.publishers_.append(p)
        return p

    def create_subscription(self, msg_type, topic, callback, qos_profile, *, callback_group=None,
                            event_callbacks=None, qos_overriding_options=None, raw=False):
        _check_qos(qos_profile)
        _check_topic(topic)
        if not callable(callback):
            raise TypeError('subscription callback must be callable')
        s = Subscription(self, msg_type, topic, callback, qos_profile)
        self.subscriptions_.append(s)
        return s

    def create_timer(self, timer_period_sec, callback, callback_group=None, clock=None):
        t = Timer(timer_period_sec, callback)
        self.timers_.append(t)
        return t

    def create_service(self, srv_type, srv_name, callback, *, qos_profile=None, callback_group=None):
        _check_topic(srv_name, 'service')
        s = Service(srv_type, srv_name, callback)
        self.services_.append(s)
        return s

    def create_client(self, srv_type, srv_name, *, qos_profile=None, callback_group=None):
        _check_topic(srv_name, 'service')
        c = Client(srv_type, srv_name)
        self.clients_.append(c)
        return c

    def create_rate(self, frequency, clock=None):
        return types.SimpleNamespace(sleep=lambda: _time.sleep(min(1.0 / frequency, 0.01)),
                                     destroy=lambda: None)

    def destroy_publisher(self, p):
        return True

    def destroy_subscription(self, s):
        return True

    def destroy_timer(self, t):
        t.cancel()
        return True

    def destroy_service(self, s):
        return True

    def destroy_client(self, c):
        return True

    def destroy_node(self):
        self.destroyed = True
        for t in self.timers_:
            t.cancel()

    def get_topic_names_and_types(self, no_demangle=False):
        return []

    def count_publishers(self, topic_name):
        return 0

    def count_subscribers(self, topic_name):
        return 0

    # --- test helpers (not rclpy API)
    def tick(self, rounds=1):
        """Run every active timer callback ``rounds`` times, like a spinning executor.

        Waits one timer period (capped at TICK_MAX_WAIT) before each round, so callbacks
        that rate-limit themselves with time.monotonic() see real time pass.
        """
        for _ in range(rounds):
            active = [t for t in self.timers_ if not t.canceled]
            if active:
                _sleep(min(max(t.timer_period_ns for t in active) / 1e9, TICK_MAX_WAIT))
            for t in active:
                if not t.canceled:
                    t.callback()

    def published(self):
        return sum(len(p.messages) for p in self.publishers_)


# ----------------------------------------------------------------------------- executors / top level
_context = {'ok': False}


def init(*, args=None, context=None, domain_id=None, signal_handler_options=None):
    _context['ok'] = True
    _context['spins'] = 0


def shutdown(*, context=None, uninstall_handlers=None):
    if not _context['ok']:
        raise RuntimeError('Context must be initialized before it can be shutdown')
    _context['ok'] = False


def try_shutdown(*, context=None, uninstall_handlers=None):
    _context['ok'] = False


def ok(*, context=None):
    return _context['ok']


def spin(node, executor=None):
    """Run SPIN_TICKS rounds of timers, then return (stands in for Ctrl+C)."""
    node.tick(SPIN_TICKS)


def spin_once(node, *, executor=None, timeout_sec=None):
    node.tick(1)
    # `while rclpy.ok(): rclpy.spin_once(node)` loops end after SPIN_TICKS rounds
    _context['spins'] = _context.get('spins', 0) + 1
    if _context['spins'] >= SPIN_TICKS:
        _context['ok'] = False


def spin_until_future_complete(node, future, executor=None, timeout_sec=None):
    node.tick(1)


def create_node(node_name, **kwargs):
    return Node(node_name, **kwargs)


class Executor:
    def __init__(self, *, context=None, num_threads=None):
        self._nodes = []

    def add_node(self, node):
        self._nodes.append(node)
        return True

    def remove_node(self, node):
        self._nodes.remove(node)

    def get_nodes(self):
        return list(self._nodes)

    def spin(self):
        for n in self._nodes:
            n.tick(SPIN_TICKS)

    def spin_once(self, timeout_sec=None):
        for n in self._nodes:
            n.tick(1)

    def shutdown(self, timeout_sec=None):
        return True


class SingleThreadedExecutor(Executor):
    pass


class MultiThreadedExecutor(Executor):
    pass


class _Group:
    def __init__(self, *a, **k):
        pass


class ReentrantCallbackGroup(_Group):
    pass


class MutuallyExclusiveCallbackGroup(_Group):
    pass


# ----------------------------------------------------------------------------- module assembly
class _ApiModule(types.ModuleType):
    """Exposes exactly the public names of the real rclpy module."""

    def __init__(self, name, impl):
        super().__init__(name)
        self._real = set(_API.get(name, []))
        self.__dict__.update({k: v for k, v in impl.items() if k in self._real or k.startswith('__')})

    def __getattr__(self, attr):
        if attr.startswith('__'):
            raise AttributeError(attr)
        if attr in self._real:
            raise NotImplementedError(f'rclpy stub: {self.__name__}.{attr} is not implemented')
        raise AttributeError(f"module '{self.__name__}' has no attribute '{attr}'")


def install():
    g = globals()
    impl = {
        'rclpy': {k: g[k] for k in ('init', 'shutdown', 'try_shutdown', 'ok', 'spin', 'spin_once',
                                    'spin_until_future_complete', 'create_node')},
        'rclpy.node': {'Node': Node, 'Parameter': Parameter},
        'rclpy.parameter': {'Parameter': Parameter},
        'rclpy.qos': {k: g[k] for k in ('QoSProfile', 'ReliabilityPolicy', 'HistoryPolicy',
                                        'DurabilityPolicy', 'LivelinessPolicy', 'qos_profile_sensor_data',
                                        'qos_profile_system_default', 'qos_profile_services_default',
                                        'qos_profile_parameters')},
        'rclpy.executors': {k: g[k] for k in ('Executor', 'SingleThreadedExecutor', 'MultiThreadedExecutor')},
        'rclpy.exceptions': {k: g[k] for k in (
            'ParameterException', 'ParameterNotDeclaredException', 'ParameterAlreadyDeclaredException',
            'InvalidParameterTypeException', 'InvalidParameterValueException',
            'ParameterUninitializedException', 'InvalidTopicNameException', 'InvalidNodeNameException',
            'ROSInterruptException')},
        'rclpy.duration': {'Duration': Duration},
        'rclpy.time': {'Time': Time},
        'rclpy.clock': {'Clock': Clock},
        'rclpy.callback_groups': {'ReentrantCallbackGroup': ReentrantCallbackGroup,
                                  'MutuallyExclusiveCallbackGroup': MutuallyExclusiveCallbackGroup},
        'rclpy.logging': {'get_logger': get_logger},
        'rclpy.timer': {'Timer': Timer},
        'rclpy.publisher': {'Publisher': Publisher},
        'rclpy.subscription': {'Subscription': Subscription},
        'rclpy.service': {'Service': Service},
        'rclpy.client': {'Client': Client},
    }
    impl['rclpy.qos'].update({'QoSReliabilityPolicy': ReliabilityPolicy, 'QoSHistoryPolicy': HistoryPolicy,
                              'QoSDurabilityPolicy': DurabilityPolicy})
    root = _ApiModule('rclpy', impl['rclpy'])
    root.__path__ = []
    sys.modules['rclpy'] = root
    for name, members in impl.items():
        if name == 'rclpy':
            continue
        mod = _ApiModule(name, members)
        sys.modules[name] = mod
        object.__setattr__(root, name.split('.', 1)[1], mod)
        root._real.add(name.split('.', 1)[1])
