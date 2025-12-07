import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure
from xpi_commons.i2c_helper import get_smbus
import time
import struct
import math

class BME280Node(Node):
    """
    ROS2 Node for the BME280 environmental sensor.
    Reads temperature, humidity, and pressure via I2C.
    Publishes sensor_msgs/Temperature, sensor_msgs/RelativeHumidity, sensor_msgs/FluidPressure.
    """

    # BME280 Registers
    BME280_REGISTER_DIG_T1 = 0x88
    BME280_REGISTER_DIG_T2 = 0x8A
    BME280_REGISTER_DIG_T3 = 0x8C
    
    BME280_REGISTER_DIG_P1 = 0x8E
    BME280_REGISTER_DIG_P2 = 0x90
    BME280_REGISTER_DIG_P3 = 0x92
    BME280_REGISTER_DIG_P4 = 0x94
    BME280_REGISTER_DIG_P5 = 0x96
    BME280_REGISTER_DIG_P6 = 0x98
    BME280_REGISTER_DIG_P7 = 0x9A
    BME280_REGISTER_DIG_P8 = 0x9C
    BME280_REGISTER_DIG_P9 = 0x9E
    
    BME280_REGISTER_DIG_H1 = 0xA1
    BME280_REGISTER_DIG_H2 = 0xE1
    BME280_REGISTER_DIG_H3 = 0xE3
    BME280_REGISTER_DIG_H4 = 0xE4
    BME280_REGISTER_DIG_H5 = 0xE5
    BME280_REGISTER_DIG_H6 = 0xE7

    BME280_REGISTER_CHIPID = 0xD0 # Expected value 0x60
    BME280_REGISTER_VERSION = 0xD1
    BME280_REGISTER_SOFTRESET = 0xE0
    
    BME280_REGISTER_CAL26 = 0xE1  # R calibration stored in 0xE1-0xF0
    
    BME280_REGISTER_CONTROLHUMID = 0xF2
    BME280_REGISTER_STATUS = 0xF3
    BME280_REGISTER_CONTROL = 0xF4
    BME280_REGISTER_CONFIG = 0xF5
    BME280_REGISTER_PRESSDATA = 0xF7
    BME280_REGISTER_TEMPDATA = 0xFA
    BME280_REGISTER_HUMIDDATA = 0xFD

    # Oversampling settings
    OVERSAMPLING_SKIPPED = 0x00
    OVERSAMPLING_1X = 0x01
    OVERSAMPLING_2X = 0x02
    OVERSAMPLING_4X = 0x03
    OVERSAMPLING_8X = 0x04
    OVERSAMPLING_16X = 0x05

    # Power modes
    MODE_SLEEP = 0x00
    MODE_FORCED = 0x01
    MODE_NORMAL = 0x03

    def __init__(self):
        super().__init__('bme280_node')

        # 1. Declare Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x76) # Default for BME280, can be 0x77
        self.declare_parameter('publish_rate', 1.0) # Hz
        self.declare_parameter('osrs_t', self.OVERSAMPLING_2X) # Oversampling temp
        self.declare_parameter('osrs_p', self.OVERSAMPLING_16X) # Oversampling pressure
        self.declare_parameter('osrs_h', self.OVERSAMPLING_1X) # Oversampling humidity
        self.declare_parameter('mode', self.MODE_NORMAL) # Power mode
        self.declare_parameter('frame_id', 'bme280_link')
        self.declare_parameter('mock_hardware', False)

        # 2. Read Parameters
        self.bus_id = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.osrs_t = self.get_parameter('osrs_t').value
        self.osrs_p = self.get_parameter('osrs_p').value
        self.osrs_h = self.get_parameter('osrs_h').value
        self.mode = self.get_parameter('mode').value
        self.frame_id = self.get_parameter('frame_id').value
        mock_mode = self.get_parameter('mock_hardware').value

        # 3. Init I2C
        self.bus = get_smbus(self.bus_id, mock=mock_mode)
        
        self.calibration_params = {}

        try:
            self.init_bme280()
            self.get_logger().info(f'BME280 initialized at 0x{self.address:02X} on bus {self.bus_id}.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize BME280: {e}. Falling back to mock.')
            self.bus.close() # Ensure mock bus is used if real fails
            self.bus = get_smbus(self.bus_id, mock=True) # Re-init as mock
            mock_mode = True

        if mock_mode:
            self.get_logger().warn('BME280: Running in MOCK mode. No real I2C data will be read.')
            self.mock_temp = 22.5
            self.mock_humidity = 60.0
            self.mock_pressure = 1013.25
            self.mock_time = time.monotonic()

        # 4. Publishers
        self.temp_publisher = self.create_publisher(Temperature, '~/temperature', 10)
        self.hum_publisher = self.create_publisher(RelativeHumidity, '~/humidity', 10)
        self.press_publisher = self.create_publisher(FluidPressure, '~/pressure', 10)
        
        # 5. Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(f'BME280: Publishing data at {self.publish_rate} Hz.')

    def _read_calibration_params(self):
        # Read temperature calibration
        self.calibration_params['dig_T1'] = self._read_word_little_endian(self.BME280_REGISTER_DIG_T1)
        self.calibration_params['dig_T2'] = self._read_signed_word_little_endian(self.BME280_REGISTER_DIG_T2)
        self.calibration_params['dig_T3'] = self._read_signed_word_little_endian(self.BME280_REGISTER_DIG_T3)

        # Read pressure calibration
        self.calibration_params['dig_P1'] = self._read_word_little_endian(self.BME280_REGISTER_DIG_P1)
        self.calibration_params['dig_P2'] = self._read_signed_word_little_endian(self.BME280_REGISTER_DIG_P2)
        self.calibration_params['dig_P3'] = self._read_signed_word_little_endian(self.BME280_REGISTER_DIG_P3)
        self.calibration_params['dig_P4'] = self._read_signed_word_little_endian(self.BME280_REGISTER_DIG_P4)
        self.calibration_params['dig_P5'] = self._read_signed_word_little_endian(self.BME280_REGISTER_DIG_P5)
        self.calibration_params['dig_P6'] = self._read_signed_word_little_endian(self.BME280_REGISTER_DIG_P6)
        self.calibration_params['dig_P7'] = self._read_signed_word_little_endian(self.BME280_REGISTER_DIG_P7)
        self.calibration_params['dig_P8'] = self._read_signed_word_little_endian(self.BME280_REGISTER_DIG_P8)
        self.calibration_params['dig_P9'] = self._read_signed_word_little_endian(self.BME280_REGISTER_DIG_P9)

        # Read humidity calibration
        self.calibration_params['dig_H1'] = self.bus.read_byte_data(self.address, self.BME280_REGISTER_DIG_H1)
        self.calibration_params['dig_H2'] = self._read_signed_word_little_endian(self.BME280_REGISTER_DIG_H2)
        self.calibration_params['dig_H3'] = self.bus.read_byte_data(self.address, self.BME280_REGISTER_DIG_H3)
        self.calibration_params['dig_H4'] = (self.bus.read_byte_data(self.address, self.BME280_REGISTER_DIG_H4) << 4) | (self.bus.read_byte_data(self.address, self.BME280_REGISTER_DIG_H5) & 0x0F)
        self.calibration_params['dig_H5'] = (self.bus.read_byte_data(self.address, self.BME280_REGISTER_DIG_H6) << 4) | (self.bus.read_byte_data(self.address, self.BME280_REGISTER_DIG_H5) >> 4)
        self.calibration_params['dig_H6'] = self._read_signed_byte(self.address, self.BME280_REGISTER_DIG_H7)

    def _read_byte(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def _read_signed_byte(self, reg):
        val = self._read_byte(reg)
        if val & 0x80:
            return -((~val & 0xFF) + 1)
        return val

    def _read_word_little_endian(self, reg):
        low = self.bus.read_byte_data(self.address, reg)
        high = self.bus.read_byte_data(self.address, reg + 1)
        return (high << 8) | low

    def _read_signed_word_little_endian(self, reg):
        val = self._read_word_little_endian(reg)
        if val & 0x8000:
            return -((~val & 0xFFFF) + 1)
        return val

    def init_bme280(self):
        # Read Chip ID
        chip_id = self.bus.read_byte_data(self.address, self.BME280_REGISTER_CHIPID)
        if chip_id != 0x60:
            raise Exception(f"BME280 Chip ID mismatch: Expected 0x60, got 0x{chip_id:02X}")
        
        # Reset the device
        self.bus.write_byte_data(self.address, self.BME280_REGISTER_SOFTRESET, 0xB6)
        time.sleep(0.01) # Wait for reset
        
        # Read calibration parameters
        self._read_calibration_params()
        
        # Set humidity oversampling
        self.bus.write_byte_data(self.address, self.BME280_REGISTER_CONTROLHUMID, self.osrs_h)
        
        # Set temperature and pressure oversampling and power mode
        ctrl_meas = (self.osrs_t << 5) | (self.osrs_p << 2) | self.mode
        self.bus.write_byte_data(self.address, self.BME280_REGISTER_CONTROL, ctrl_meas)
        time.sleep(0.01) # Give it time to start a measurement if in normal mode

    def _compensate_temperature(self, adc_T):
        dig_T1 = self.calibration_params['dig_T1']
        dig_T2 = self.calibration_params['dig_T2']
        dig_T3 = self.calibration_params['dig_T3']

        var1 = ((adc_T / 16384.0) - (dig_T1 / 1024.0)) * dig_T2
        var2 = (((adc_T / 131072.0) - (dig_T1 / 8192.0)) * ((adc_T / 131072.0) - (dig_T1 / 8192.0))) * dig_T3
        self.t_fine = var1 + var2
        temperature_c = self.t_fine / 5120.0
        return temperature_c

    def _compensate_pressure(self, adc_P):
        dig_P1 = self.calibration_params['dig_P1']
        dig_P2 = self.calibration_params['dig_P2']
        dig_P3 = self.calibration_params['dig_P3']
        dig_P4 = self.calibration_params['dig_P4']
        dig_P5 = self.calibration_params['dig_P5']
        dig_P6 = self.calibration_params['dig_P6']
        dig_P7 = self.calibration_params['dig_P7']
        dig_P8 = self.calibration_params['dig_P8']
        dig_P9 = self.calibration_params['dig_P9']

        var1 = (self.t_fine / 2.0) - 64000.0
        var2 = (((var1 / 4.0) * (var1 / 4.0)) / 2048.0) * dig_P6
        var2 = var2 + ((var1 * dig_P5) * 2.0)
        var2 = (var2 / 4.0) + (dig_P4 * 65536.0)
        var1 = (((dig_P3 * (((var1 / 4.0) * (var1 / 4.0)) / 8192.0)) / 8.0) + ((dig_P2 * var1) / 2.0)) / 262144.0
        var1 = ((32768.0 + var1) * dig_P1) / 32768.0
        if var1 == 0:
            return 0  # Avoid division by zero
        pressure_pa = ((1048576.0 - adc_P) - (var2 / 4096.0)) * 6250.0 / var1
        var1 = (dig_P9 * (((pressure_pa / 256.0) * (pressure_pa / 256.0)) / 8192.0)) / 16.0
        var2 = ((pressure_pa * dig_P8) / 32.0)
        pressure_pa = pressure_pa + ((var1 + var2 + dig_P7) / 16.0)
        return pressure_pa

    def _compensate_humidity(self, adc_H):
        dig_H1 = self.calibration_params['dig_H1']
        dig_H2 = self.calibration_params['dig_H2']
        dig_H3 = self.calibration_params['dig_H3']
        dig_H4 = self.calibration_params['dig_H4']
        dig_H5 = self.calibration_params['dig_H5']
        dig_H6 = self.calibration_params['dig_H6']
        
        var_H = (self.t_fine - 76800.0)
        var_H = (adc_H - (dig_H4 * 64.0 + dig_H5 / 16384.0 * var_H)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * var_H * (1.0 + dig_H3 / 67108864.0 * var_H)))
        var_H = var_H * (1.0 - dig_H1 * var_H / 524288.0)
        if var_H > 100.0:
            var_H = 100.0
        elif var_H < 0.0:
            var_H = 0.0
        return var_H # returns humidity in %RH

    def timer_callback(self):
        temp_msg = Temperature()
        hum_msg = RelativeHumidity()
        press_msg = FluidPressure()
        
        current_time = self.get_clock().now().to_msg()
        temp_msg.header.stamp = current_time
        hum_msg.header.stamp = current_time
        press_msg.header.stamp = current_time

        temp_msg.header.frame_id = self.frame_id
        hum_msg.header.frame_id = self.frame_id
        press_msg.header.frame_id = self.frame_id

        if self.bus.mock_mode:
            t = (time.monotonic() - self.mock_time) % 60.0 # Cycle every minute
            
            temp_msg.temperature = 22.5 + 2.5 * math.sin(t * math.pi / 15.0) # 20-25 degC
            hum_msg.relative_humidity = 0.6 + 0.1 * math.cos(t * math.pi / 20.0) # 50-70%
            press_msg.fluid_pressure = 101325.0 + 500.0 * math.sin(t * math.pi / 25.0) # Pascals (1013.25 mbar)

            self.get_logger().debug(f'Mock: T={temp_msg.temperature:.2f}C, H={hum_msg.relative_humidity*100:.1f}%, P={press_msg.fluid_pressure/100:.2f}hPa')
            
        else:
            try:
                # Read raw sensor data
                # Temp data
                xlsb = self.bus.read_byte_data(self.address, self.BME280_REGISTER_TEMPDATA)
                lsb = self.bus.read_byte_data(self.address, self.BME280_REGISTER_TEMPDATA + 1)
                msb = self.bus.read_byte_data(self.address, self.BME280_REGISTER_TEMPDATA + 2)
                adc_T = (msb << 12) | (lsb << 4) | (xlsb >> 4)

                # Pressure data
                xlsb = self.bus.read_byte_data(self.address, self.BME280_REGISTER_PRESSDATA)
                lsb = self.bus.read_byte_data(self.address, self.BME280_REGISTER_PRESSDATA + 1)
                msb = self.bus.read_byte_data(self.address, self.BME280_REGISTER_PRESSDATA + 2)
                adc_P = (msb << 12) | (lsb << 4) | (xlsb >> 4)
                
                # Humidity data
                lsb = self.bus.read_byte_data(self.address, self.BME280_REGISTER_HUMIDDATA)
                msb = self.bus.read_byte_data(self.address, self.BME280_REGISTER_HUMIDDATA + 1)
                adc_H = (msb << 8) | lsb

                # Compensate and get final values
                temp_msg.temperature = self._compensate_temperature(adc_T)
                press_pa = self._compensate_pressure(adc_P)
                press_msg.fluid_pressure = press_pa # Pascals
                hum_msg.relative_humidity = self._compensate_humidity(adc_H) / 100.0 # Convert %RH to [0,1]

                self.get_logger().debug(f'Real: T={temp_msg.temperature:.2f}C, H={hum_msg.relative_humidity*100:.1f}%, P={press_msg.fluid_pressure/100:.2f}hPa')

            except Exception as e:
                self.get_logger().error(f'BME280: Error reading sensor data: {e}')
                return # Skip publishing if error

        self.temp_publisher.publish(temp_msg)
        self.hum_publisher.publish(hum_msg)
        self.press_publisher.publish(press_msg)

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BME280Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
