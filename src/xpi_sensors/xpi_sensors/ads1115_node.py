import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from xpi_commons.i2c_helper import get_smbus
import time
import struct
import math

class ADS1115Node(Node):
    """
    ROS2 Node for the ADS1115 16-bit ADC.
    Reads analog values from up to 4 channels via I2C and publishes them as raw voltages.
    Supports single-ended and differential measurements.
    """

    # ADS1115 Registers
    REG_CONVERT = 0x00
    REG_CONFIG = 0x01
    REG_LOWTHRES = 0x02
    REG_HITHRES = 0x03

    # Config Register Bit Masks
    OS_SINGLE = 0x8000 # OS Bit 15: 1 for single conversion
    
    MUX_OFFSET = 12 # MUX Bits 14:12 (A0 to A3, or differential)
    MUX = {
        '0_GND': 0x4000, '1_GND': 0x5000, '2_GND': 0x6000, '3_GND': 0x7000, # Single-ended
        '0_1': 0x0000, '0_3': 0x1000, '1_3': 0x2000, '2_3': 0x3000 # Differential
    }

    PGA_OFFSET = 9 # PGA Bits 11:9
    PGA = { # Gain, Vmax
        '2/3': (0x0000, 6.144),  # +/-6.144V
        '1':   (0x0200, 4.096),  # +/-4.096V
        '2':   (0x0400, 2.048),  # +/-2.048V
        '4':   (0x0600, 1.024),  # +/-1.024V
        '8':   (0x0800, 0.512),  # +/-0.512V
        '16':  (0x0A00, 0.256)   # +/-0.256V
    }

    MODE_SINGLE = 0x0100 # MODE Bit 8: 1 for single-shot, 0 for continuous
    MODE_CONTINUOUS = 0x0000

    DR_OFFSET = 5 # DR Bits 7:5
    DR = { # Data Rate (samples per second)
        8: 0x0000, 16: 0x0020, 32: 0x0040, 64: 0x0060,
        128: 0x0080, 250: 0x00A0, 475: 0x00C0, 860: 0x00E0
    }

    COMP_MODE_WINDOW = 0x0010
    COMP_MODE_TRAD = 0x0000

    COMP_POL_ACTIVE_LOW = 0x0008
    COMP_POL_ACTIVE_HIGH = 0x0000

    COMP_LAT_LATCHING = 0x0004
    COMP_LAT_NONLATCHING = 0x0000

    COMP_QUE = { # Comparator Queue
        '1_CONV': 0x0000, '2_CONV': 0x0001, '4_CONV': 0x0002, 'DISABLE': 0x0003
    }

    def __init__(self):
        super().__init__('ads1115_node')

        # 1. Declare Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x48) # Default for ADS1115, can be 0x49, 0x4A, 0x4B
        self.declare_parameter('publish_rate', 10.0) # Hz
        self.declare_parameter('channels', [0]) # List of single-ended channels to read (0-3)
        self.declare_parameter('pga', '2/3') # Programmable Gain Amplifier setting
        self.declare_parameter('data_rate', 128) # Samples per second (8, 16, 32, 64, 128, 250, 475, 860)
        self.declare_parameter('frame_id', 'ads1115_link')
        self.declare_parameter('mock_hardware', False)

        # 2. Read Parameters
        self.bus_id = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.channels = self.get_parameter('channels').value
        self.pga_setting = str(self.get_parameter('pga').value)
        self.data_rate = self.get_parameter('data_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        mock_mode = self.get_parameter('mock_hardware').value

        # Validate channels
        for ch in self.channels:
            if not (0 <= ch <= 3):
                self.get_logger().error(f"Invalid channel {ch}. Must be between 0 and 3.")
                self.channels = [0] # Default to A0
                break
        
        # Validate PGA
        if self.pga_setting not in self.PGA:
            self.get_logger().error(f"Invalid PGA setting: {self.pga_setting}. Defaulting to '2/3'.")
            self.pga_setting = '2/3'
        
        # Validate Data Rate
        if self.data_rate not in self.DR:
            self.get_logger().error(f"Invalid data rate: {self.data_rate}. Defaulting to 128 SPS.")
            self.data_rate = 128


        # 3. Init I2C
        self.bus = get_smbus(self.bus_id, mock=mock_mode)
        
        self.v_ref = self.PGA[self.pga_setting][1] # Max voltage based on PGA
        self.lsb = self.v_ref / 32768.0 # LSB value (Volts per bit)

        self.mock_channel_values = [0.0] * 4 # For mock mode
        self.mock_time = time.monotonic()

        try:
            self._write_config(self.REG_CONFIG, self.OS_SINGLE | self.MUX['0_GND'] | self.PGA[self.pga_setting][0] | self.DR[self.data_rate] | self.MODE_SINGLE | self.COMP_QUE['DISABLE'])
            self.get_logger().info(f'ADS1115 initialized at 0x{self.address:02X} on bus {self.bus_id}.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ADS1115: {e}. Falling back to mock.')
            self.bus.close() # Ensure mock bus is used if real fails
            self.bus = get_smbus(self.bus_id, mock=True) # Re-init as mock
            mock_mode = True

        if mock_mode:
            self.get_logger().warn('ADS1115: Running in MOCK mode. No real I2C data will be read.')

        # 4. Publishers
        self.channel_publishers = {}
        for ch in self.channels:
            self.channel_publishers[ch] = self.create_publisher(Float32, f'~/voltage_ch{ch}', 10)
        
        # Publisher for all channels in one array
        if len(self.channels) > 1:
            self.all_channels_publisher = self.create_publisher(Float32MultiArray, '~/all_voltages', 10)
        else:
            self.all_channels_publisher = None
        
        # 5. Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(f'ADS1115: Publishing data at {self.publish_rate} Hz.')

    def _write_config(self, register, value):
        # ADS1115 expects MSB first
        high_byte = (value >> 8) & 0xFF
        low_byte = value & 0xFF
        self.bus.write_i2c_block_data(self.address, register, [high_byte, low_byte])

    def _read_conversion_result(self):
        # ADS1115 returns MSB first
        raw = self.bus.read_i2c_block_data(self.address, self.REG_CONVERT, 2)
        value = struct.unpack('>h', bytes(raw))[0] # Read as signed short (big-endian)
        return value

    def _get_mux_for_channel(self, channel: int):
        if channel == 0: return self.MUX['0_GND']
        if channel == 1: return self.MUX['1_GND']
        if channel == 2: return self.MUX['2_GND']
        if channel == 3: return self.MUX['3_GND']
        return self.MUX['0_GND'] # Default

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg()
        all_voltages = []

        for ch in self.channels:
            voltage = 0.0
            if self.bus.mock_mode:
                # Simulate some varying analog data
                t = (time.monotonic() - self.mock_time)
                voltage = (math.sin(t * math.pi / (5.0 + ch)) * (self.v_ref / 3.0)) + (self.v_ref / 2.0)
                voltage = max(0.0, min(self.v_ref, voltage)) # Clamp to 0-Vref
                self.mock_channel_values[ch] = voltage
                self.get_logger().debug(f"Mock ch{ch}: {voltage:.3f}V")
            else:
                try:
                    # Configure for single-ended read on current channel
                    config = (
                        self.OS_SINGLE |
                        self._get_mux_for_channel(ch) |
                        self.PGA[self.pga_setting][0] |
                        self.DR[self.data_rate] |
                        self.MODE_SINGLE |
                        self.COMP_QUE['DISABLE']
                    )
                    self._write_config(self.REG_CONFIG, config)
                    
                    # Wait for conversion to complete (max 8ms for 8 SPS, ~1ms for 860 SPS)
                    # For self.publish_rate, we assume conversion will complete within timer interval
                    time.sleep(1.0 / self.data_rate + 0.0001) # Small buffer
                    
                    # Read conversion status
                    status = self.bus.read_word_data(self.address, self.REG_CONFIG)
                    while not (status & self.OS_SINGLE): # Wait until OS bit is 1
                         time.sleep(0.0001)
                         status = self.bus.read_word_data(self.address, self.REG_CONFIG)

                    raw_value = self._read_conversion_result()
                    voltage = raw_value * self.lsb
                    # self.get_logger().debug(f"Ch{ch}: Raw={raw_value}, V={voltage:.3f}V")

                except Exception as e:
                    self.get_logger().error(f'ADS1115: Error reading channel {ch}: {e}')
                    voltage = 0.0 # Default to 0V on error
            
            # Publish individual channel voltage
            msg = Float32()
            msg.data = voltage
            self.channel_publishers[ch].publish(msg)
            all_voltages.append(voltage)
        
        # Publish all channels in one array
        if self.all_channels_publisher:
            multi_msg = Float32MultiArray()
            multi_msg.data = all_voltages
            self.all_channels_publisher.publish(multi_msg)

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ADS1115Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
