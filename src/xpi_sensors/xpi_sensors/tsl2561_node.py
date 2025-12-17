#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Illuminance
from xpi_commons.i2c_helper import get_smbus
import time

class TSL2561Node(Node):
    def __init__(self):
        super().__init__('tsl2561_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x39)  # Default: 0x39 (Float), 0x29 (GND), 0x49 (VCC)
        self.declare_parameter('poll_rate', 1.0)     # Hz
        self.declare_parameter('frame_id', 'tsl2561_link')
        self.declare_parameter('gain_16x', False)    # False=1x (Low), True=16x (High)
        self.declare_parameter('integration_time', 2) # 0=13.7ms, 1=101ms, 2=402ms

        self.i2c_bus_id = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.poll_rate = self.get_parameter('poll_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.gain_16x = self.get_parameter('gain_16x').value
        self.integ_time = self.get_parameter('integration_time').value

        # TSL2561 Registers (Command bit 0x80 must be set for all transactions)
        self.COMMAND_BIT = 0x80
        self.WORD_BIT = 0x20
        self.CONTROL_POWERON = 0x03
        self.CONTROL_POWEROFF = 0x00
        
        self.REG_CONTROL = 0x00
        self.REG_TIMING = 0x01
        self.REG_DATA_0_LOW = 0x0C
        self.REG_DATA_1_LOW = 0x0E

        # I2C Setup
        try:
            self.bus = get_smbus(self.i2c_bus_id)
            self.configure_sensor()
            self.get_logger().info(f"TSL2561 initialized on bus {self.i2c_bus_id} at address {hex(self.i2c_address)}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize TSL2561: {e}")
            self.destroy_node()
            return

        # Publishers
        self.pub_lux = self.create_publisher(Illuminance, '~/illuminance', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.poll_rate, self.read_sensor)

    def write_reg(self, reg, value):
        self.bus.write_byte_data(self.i2c_address, self.COMMAND_BIT | reg, value)

    def read_word(self, reg):
        # Read 16-bit word with Command+Word bits set
        return self.bus.read_word_data(self.i2c_address, self.COMMAND_BIT | self.WORD_BIT | reg)

    def configure_sensor(self):
        # Power Up
        self.write_reg(self.REG_CONTROL, self.CONTROL_POWERON)
        
        # Timing & Gain
        # Register 0x01: | - | - | - | Gain | Manual | Reserved | Integ1 | Integ0 |
        # Gain: 0=1x, 1=16x
        # Integ: 00=13.7ms, 01=101ms, 10=402ms
        
        gain_bit = 0x10 if self.gain_16x else 0x00
        integ_bits = self.integ_time & 0x03
        timing_val = gain_bit | integ_bits
        
        self.write_reg(self.REG_TIMING, timing_val)

    def calculate_lux(self, ch0, ch1):
        # Simplified Lux Calculation (CS Package, T, FN, and CL Package)
        # Based on Datasheet logic
        
        if ch0 == 0:
            return 0.0

        ratio = ch1 / ch0
        
        # Scaling based on Integration Time
        # Default is 402ms (Scale = 1.0). If faster, we need to scale up raw values if we want raw normalization,
        # but the Lux formula usually relies on the ratio, which is time-invariant.
        # However, absolute counts depend on time.
        # The standard C code libraries usually normalize counts to 402ms equivalents before formula, 
        # or use different coefficients. 
        # Here we use a standard approximation for 402ms/16x or adapt.
        # Actually, simpler method: Use the Adafruit/Sparkfun logic which handles scale.
        
        # Normalizing counts to 402ms and 16x gain equivalent is good practice, 
        # but let's stick to the ratio formula which handles the physics.
        
        # Coefficients for T, FN, CL packages (Default mostly)
        lux = 0.0
        
        if 0 < ratio <= 0.50:
            lux = 0.0304 * ch0 - 0.062 * ch0 * (ratio**1.4)
        elif 0.50 < ratio <= 0.61:
            lux = 0.0224 * ch0 - 0.031 * ch1
        elif 0.61 < ratio <= 0.80:
            lux = 0.0128 * ch0 - 0.0153 * ch1
        elif 0.80 < ratio <= 1.30:
            lux = 0.00146 * ch0 - 0.00112 * ch1
        else:
            lux = 0.0
            
        # Adjust for Gain and Time if they are NOT standard settings in the formula assumptions?
        # The formula above typically assumes no scaling needed if inputs are raw counts, 
        # BUT the constants change based on integration time/gain scaling.
        # Let's apply a simple scalar correction if we are not at 402ms/16x?
        # Actually, standard formula works on RAW counts provided they are not saturated.
        # We just need to handle clipping.
        
        # Scale adjustment factor
        # If Integ = 13ms, counts are very low -> Lux formula yields low value.
        # We need to scale calculated Lux by (402 / current_ms) and (16 / current_gain)?
        # Let's verify standard lib behavior: usually they Normalize Raw Data first.
        
        # Scale map
        ms_map = {0: 13.7, 1: 101.0, 2: 402.0}
        integ_ms = ms_map.get(self.integ_time, 402.0)
        
        # If gain is 1x (Low), we need to multiply result by 16 compared to High Gain context?
        # Or multiply RAW data by 16?
        # Let's Normalize RAW data to "Nominal 402ms, 16x" equivalent roughly.
        
        scale = 402.0 / integ_ms
        if not self.gain_16x:
            scale *= 16
            
        # Apply scaling to raw counts
        ch0_norm = ch0 * scale
        ch1_norm = ch1 * scale
        
        # Recalculate ratio with normalized
        # Ratio is identical (scale cancels out), but Ch0 magnitude changes.
        
        if 0 < ratio <= 0.50:
            lux = 0.0304 * ch0_norm - 0.062 * ch0_norm * (ratio**1.4)
        elif 0.50 < ratio <= 0.61:
            lux = 0.0224 * ch0_norm - 0.031 * ch1_norm
        elif 0.61 < ratio <= 0.80:
            lux = 0.0128 * ch0_norm - 0.0153 * ch1_norm
        elif 0.80 < ratio <= 1.30:
            lux = 0.00146 * ch0_norm - 0.00112 * ch1_norm
        else:
            lux = 0.0
            
        return max(0.0, lux)

    def read_sensor(self):
        try:
            # Read Channel 0 (Visible + IR) and Channel 1 (IR)
            ch0 = self.read_word(self.REG_DATA_0_LOW)
            ch1 = self.read_word(self.REG_DATA_1_LOW)
            
            lux = self.calculate_lux(ch0, ch1)
            
            msg = Illuminance()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.illuminance = float(lux)
            msg.variance = 0.0 
            
            # Simple debug log for validation
            # self.get_logger().info(f"Ch0: {ch0}, Ch1: {ch1}, Lux: {lux:.2f}")
            
            self.pub_lux.publish(msg)
            
        except Exception as e:
            self.get_logger().warning(f"Error reading TSL2561: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TSL2561Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
