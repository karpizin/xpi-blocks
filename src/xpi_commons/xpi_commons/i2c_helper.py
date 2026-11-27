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
        mock (bool): Force mock mode.
    
    Returns:
        SMBus-like object with a boolean ``mock_mode`` attribute
        (True for MockSMBus, False for a real smbus2.SMBus).
    """
    # Auto-detect mock requirement
    force_mock = mock or os.environ.get('XPI_MOCK_I2C') == '1'
    
    if not force_mock and SMBus is not None:
        try:
            bus = SMBus(bus_id)
            bus.mock_mode = False
            return bus
        except (FileNotFoundError, PermissionError) as e:
            # Fallback if /dev/i2c-x doesn't exist (e.g. running on PC without mock flag)
            logging.getLogger('xpi_commons').warning(f"Failed to open /dev/i2c-{bus_id}: {e}. Falling back to MOCK.")
            return MockSMBus(bus_id)
    
    return MockSMBus(bus_id)
