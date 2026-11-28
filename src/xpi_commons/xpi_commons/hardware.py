"""Failing loudly when hardware is missing, instead of publishing simulated data.

A node that silently switches to mock data on a hardware error publishes values
that look real (plausible temperatures, IMU readings, RC sticks). Anything
downstream, an LLM agent included, then acts on fake data. Simulation is
therefore only used when it is asked for explicitly with ``mock_hardware:=true``.
"""

MOCK_HINT = ('Check the wiring and that the interface (I2C/SPI/UART/1-Wire) is enabled, '
             'or start the node with mock_hardware:=true to run on simulated data.')


class HardwareUnavailableError(RuntimeError):
    """Hardware could not be initialized and mock mode was not requested."""


def hardware_unavailable(node, what, exc=None):
    """Log a fatal error on ``node`` and raise HardwareUnavailableError.

    ``what`` says what failed, e.g. ``'AHT20 at 0x38 on I2C bus 1'``.
    """
    msg = f'{what}: {exc}' if exc is not None else what
    msg = f'{msg}. Not falling back to simulated data. {MOCK_HINT}'
    if node is not None:
        node.get_logger().fatal(msg)
    raise HardwareUnavailableError(msg) from exc
