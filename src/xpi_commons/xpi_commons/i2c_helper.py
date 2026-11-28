import os
import logging

from .hardware import HardwareUnavailableError, MOCK_HINT

# Try to import smbus2, handle failure gracefully for non-Linux/dev environments
try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

class MockSMBus:
    """
    A simulation of the SMBus for development on non-Pi hardware (CI/CD, Mac, Windows).
    It logs register writes and returns dummy data.

    Nodes check ``bus.mock_mode`` to decide whether to talk to hardware
    or generate simulated readings.
    """
    mock_mode = True

    def __init__(self, bus_id=1):
        self.bus_id = bus_id
        self.registers = {}
        self.logger = logging.getLogger('MockSMBus')
        self.logger.warning(f'Initialized MockSMBus on bus {bus_id}. No real I2C hardware is used.')

    def write_byte_data(self, i2c_addr, register, value):
        key = (i2c_addr, register)
        self.registers[key] = value
        # self.logger.info(f'I2C Write: Addr=0x{i2c_addr:02X} Reg=0x{register:02X} Val=0x{value:02X}')

    def read_byte_data(self, i2c_addr, register):
        key = (i2c_addr, register)
        val = self.registers.get(key, 0x00)
        # self.logger.info(f'I2C Read:  Addr=0x{i2c_addr:02X} Reg=0x{register:02X} -> 0x{val:02X}')
        return val

    def read_byte(self, i2c_addr):
        return self.registers.get((i2c_addr, None), 0x00)

    def write_byte(self, i2c_addr, value):
        self.registers[(i2c_addr, None)] = value

    def read_word_data(self, i2c_addr, register):
        return self.registers.get((i2c_addr, register), 0x0000)

    def write_word_data(self, i2c_addr, register, value):
        self.registers[(i2c_addr, register)] = value

    def read_i2c_block_data(self, i2c_addr, register, length):
        return [self.registers.get((i2c_addr, register + i), 0x00) for i in range(length)]

    def write_i2c_block_data(self, i2c_addr, register, data):
        for i, value in enumerate(data):
            self.registers[(i2c_addr, register + i)] = value

    def i2c_rdwr(self, *msgs):
        pass

    def close(self):
        pass
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        pass

def get_smbus(bus_id=1, mock=False):
    """
    Factory to return a real SMBus or a MockSMBus.
    
    Args:
        bus_id (int): I2C bus number (usually 1 for RPi).
        mock (bool): Use MockSMBus (also when XPI_MOCK_I2C=1).
    
    Returns:
        SMBus-like object with a boolean ``mock_mode`` attribute
        (True for MockSMBus, False for a real smbus2.SMBus).

    Raises:
        HardwareUnavailableError: the bus cannot be opened and mock was not requested.
    """
    if mock or os.environ.get('XPI_MOCK_I2C') == '1':
        return MockSMBus(bus_id)

    # No silent fallback to MockSMBus: simulated readings look real and would be
    # published as sensor data. Mock mode must be requested explicitly.
    if SMBus is None:
        raise HardwareUnavailableError(
            f'Cannot open I2C bus {bus_id}: the smbus2 package is not installed. {MOCK_HINT}')
    try:
        bus = SMBus(bus_id)
    except OSError as e:   # FileNotFoundError, PermissionError, ...
        raise HardwareUnavailableError(f'Cannot open /dev/i2c-{bus_id}: {e}. {MOCK_HINT}') from e
    bus.mock_mode = False
    return bus
