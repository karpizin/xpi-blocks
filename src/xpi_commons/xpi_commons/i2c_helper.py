import os
import logging

# Try to import smbus2, handle failure gracefully for non-Linux/dev environments
try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

class MockSMBus:
    """
    A simulation of the SMBus for development on non-Pi hardware (CI/CD, Mac, Windows).
    It logs register writes and returns dummy data.
    """
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
        mock (bool): Force mock mode.
    
    Returns:
        SMBus-like object.
    """
    # Auto-detect mock requirement
    force_mock = mock or os.environ.get('XPI_MOCK_I2C') == '1'
    
    if not force_mock and SMBus is not None:
        try:
            return SMBus(bus_id)
        except (FileNotFoundError, PermissionError) as e:
            # Fallback if /dev/i2c-x doesn't exist (e.g. running on PC without mock flag)
            logging.getLogger('xpi_commons').warning(f"Failed to open /dev/i2c-{bus_id}: {e}. Falling back to MOCK.")
            return MockSMBus(bus_id)
    
    return MockSMBus(bus_id)
