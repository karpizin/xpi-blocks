import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, FluidPressure
from xpi_commons.i2c_helper import get_smbus
import time
import struct
import math

class BMP085Node(Node):
    """
    ROS2 Node for the BMP085/BMP180 environmental sensor.
    Reads temperature and pressure via I2C.
    Publishes sensor_msgs/Temperature and sensor_msgs/FluidPressure.
    """

    # BMP085/BMP180 Registers
    BMP_ADDR = 0x77

    # Calibration registers
    AC1 = 0xAA
    AC2 = 0xAC
    AC3 = 0xAE
    AC4 = 0xB0
    AC5 = 0xB2
    AC6 = 0xB4
    B1 = 0xB6
    B2 = 0xB8
    MB = 0xBA
    MC = 0xBC
    MD = 0xBE

    # Control register
    CONTROL_REG = 0xF4
    TEMP_CMD = 0x2E  # Read temperature
    PRESS_CMD = 0x34 # Read pressure

    # Data registers
    MSB_REG = 0xF6
    LSB_REG = 0xF7
    XLSB_REG = 0xF8

    def __init__(self):
        super().__init__('bmp085_node')

        # 1. Declare Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', self.BMP_ADDR)
        self.declare_parameter('publish_rate', 1.0) # Hz
        self.declare_parameter('oss', 0) # Oversampling Setting (0-3)
        self.declare_parameter('frame_id', 'bmp085_link')
        self.declare_parameter('mock_hardware', False)

        # 2. Read Parameters
        self.bus_id = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.oss = self.get_parameter('oss').value # Oversampling setting
        self.frame_id = self.get_parameter('frame_id').value
        mock_mode = self.get_parameter('mock_hardware').value

        # 3. Init I2C
        self.bus = get_smbus(self.bus_id, mock=mock_mode)
        
        self.calibration_params = {}
        self.delay_time = 0.004 + (0.003 * (1 << self.oss)) # Delay based on OSS

        try:
            self.init_bmp085()
            self.get_logger().info(f'BMP085 initialized at 0x{self.address:02X} on bus {self.bus_id}.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BMP085: {e}. Falling back to mock.')
            self.bus.close() # Ensure mock bus is used if real fails
            self.bus = get_smbus(self.bus_id, mock=True) # Re-init as mock
            mock_mode = True

        if mock_mode:
            self.get_logger().warn('BMP085: Running in MOCK mode. No real I2C data will be read.')
            self.mock_temp = 22.0
            self.mock_pressure = 101000.0
            self.mock_time = time.monotonic()

        # 4. Publishers
        self.temp_publisher = self.create_publisher(Temperature, '~/temperature', 10)
        self.press_publisher = self.create_publisher(FluidPressure, '~/pressure', 10)
        
        # 5. Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(f'BMP085: Publishing data at {self.publish_rate} Hz.')

    def _read_s16(self, register):
        raw = self.bus.read_word_data(self.address, register)
        swapped = struct.unpack('<h', struct.pack('>h', raw))[0] # Swap bytes, then interpret as signed short
        return swapped

    def _read_u16(self, register):
        raw = self.bus.read_word_data(self.address, register)
        swapped = struct.unpack('<H', struct.pack('>H', raw))[0] # Swap bytes, then interpret as unsigned short
        return swapped

    def init_bmp085(self):
        # Read calibration data
        self.calibration_params['ac1'] = self._read_s16(self.AC1)
        self.calibration_params['ac2'] = self._read_s16(self.AC2)
        self.calibration_params['ac3'] = self._read_s16(self.AC3)
        self.calibration_params['ac4'] = self._read_u16(self.AC4)
        self.calibration_params['ac5'] = self._read_u16(self.AC5)
        self.calibration_params['ac6'] = self._read_u16(self.AC6)
        self.calibration_params['b1'] = self._read_s16(self.B1)
        self.calibration_params['b2'] = self._read_s16(self.B2)
        self.calibration_params['mb'] = self._read_s16(self.MB)
        self.calibration_params['mc'] = self._read_s16(self.MC)
        self.calibration_params['md'] = self._read_s16(self.MD)
        self.get_logger().debug(f"BMP085 calibration: {self.calibration_params}")

    def _read_raw_temp(self):
        self.bus.write_byte_data(self.address, self.CONTROL_REG, self.TEMP_CMD)
        time.sleep(self.delay_time) # Max 4.5ms for 0x2E cmd
        
        msb = self.bus.read_byte_data(self.address, self.MSB_REG)
        lsb = self.bus.read_byte_data(self.address, self.LSB_REG)
        
        return (msb << 8) | lsb

    def _read_raw_pressure(self):
        self.bus.write_byte_data(self.address, self.CONTROL_REG, self.PRESS_CMD + (self.oss << 6))
        time.sleep(self.delay_time) # Delay depends on OSS
        
        msb = self.bus.read_byte_data(self.address, self.MSB_REG)
        lsb = self.bus.read_byte_data(self.address, self.LSB_REG)
        xlsb = self.bus.read_byte_data(self.address, self.XLSB_REG)
        
        return ((msb << 16) | (lsb << 8) | xlsb) >> (8 - self.oss)

    def _compensate(self, ut, up):
        """Compensate raw temp and pressure using calibration data."""
        ac1 = self.calibration_params['ac1']
        ac2 = self.calibration_params['ac2']
        ac3 = self.calibration_params['ac3']
        ac4 = self.calibration_params['ac4']
        ac5 = self.calibration_params['ac5']
        ac6 = self.calibration_params['ac6']
        b1 = self.calibration_params['b1']
        b2 = self.calibration_params['b2']
        mb = self.calibration_params['mb']
        mc = self.calibration_params['mc']
        md = self.calibration_params['md']

        # Temperature compensation
        x1 = ((ut - ac6) * ac5) >> 15
        x2 = (mc << 11) // (x1 + md)
        b5 = x1 + x2
        temp_c = ((b5 + 8) >> 4) / 10.0 # Temperature in C

        # Pressure compensation
        b6 = b5 - 4000
        x1 = (b2 * ((b6 * b6) >> 12)) >> 11
        x2 = (ac2 * b6) >> 11
        x3 = x1 + x2
        b3 = (((ac1 * 4 + x3) << self.oss) + 2) >> 2
        x1 = (ac3 * b6) >> 13
        x2 = (b1 * ((b6 * b6) >> 12)) >> 16
        x3 = ((x1 + x2) + 2) >> 2
        b4 = (ac4 * (x3 + 32768)) >> 15
        b7 = ((up - b3) * (50000 >> self.oss))
        if b7 < 0x80000000: # Check MSB
            pressure_pa = (b7 * 2) // b4
        else:
            pressure_pa = (b7 // b4) * 2
        x1 = (pressure_pa >> 8) * (pressure_pa >> 8)
        x1 = (x1 * 3038) >> 16
        x2 = (-7357 * pressure_pa) >> 16
        pressure_pa = pressure_pa + ((x1 + x2 + 3791) >> 4) # Pressure in Pa

        return temp_c, pressure_pa

    def timer_callback(self):
        temp_msg = Temperature()
        press_msg = FluidPressure()
        
        current_time = self.get_clock().now().to_msg()
        temp_msg.header.stamp = current_time
        press_msg.header.stamp = current_time

        temp_msg.header.frame_id = self.frame_id
        press_msg.header.frame_id = self.frame_id

        if self.bus.mock_mode:
            t = (time.monotonic() - self.mock_time) % 60.0 # Cycle every minute
            
            temp_msg.temperature = 22.0 + 2.0 * math.sin(t * math.pi / 20.0) # 20-24 degC
            press_msg.fluid_pressure = 101000.0 + 1000.0 * math.cos(t * math.pi / 30.0) # Pascals (1000-1020 mbar)

            self.get_logger().debug(f'Mock: T={temp_msg.temperature:.2f}C, P={press_msg.fluid_pressure/100:.2f}hPa')
            
        else:
            try:
                raw_temp = self._read_raw_temp()
                raw_pressure = self._read_raw_pressure()
                
                temperature_c, pressure_pa = self._compensate(raw_temp, raw_pressure)

                temp_msg.temperature = temperature_c
                press_msg.fluid_pressure = pressure_pa # Pascals

                self.get_logger().debug(f'Real: T={temp_msg.temperature:.2f}C, P={press_pa/100:.2f}hPa')

            except Exception as e:
                self.get_logger().error(f'BMP085: Error reading sensor data: {e}')
                return # Skip publishing if error

        self.temp_publisher.publish(temp_msg)
        self.press_publisher.publish(press_msg)

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BMP085Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
