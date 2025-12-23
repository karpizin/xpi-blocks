import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Temperature, Illuminance, FluidPressure, Range
import json
import math
import numpy as np

# A dictionary to hold sensor configurations and their interpretation logic
# Key is the ADS1115 channel number
SENSOR_INTERPRETATION_CONFIG = {}

class AnalogSensorInterpreterNode(Node):
    """
    ROS2 Node that subscribes to ADS1115 voltage outputs and interprets them
    into physical units (e.g., Temperature, Current, Illuminance) based on
    configured sensor types.
    """

    def __init__(self):
        super().__init__('analog_sensor_interpreter_node')

        # 1. Declare Parameters
        self.declare_parameter('adc_node_name', 'ads1115_adc') # Name of the ADS1115 node
        self.declare_parameter('publish_rate', 10.0) # Hz, for publishing interpreted data
        self.declare_parameter('frame_id', 'analog_sensors_link')
        self.declare_parameter('sensor_configs_json', '[]') # JSON string of sensor configurations

        # 2. Read Parameters
        self.adc_node_name = self.get_parameter('adc_node_name').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.sensor_configs_json = self.get_parameter('sensor_configs_json').value
        
        try:
            self.sensor_configs = json.loads(self.sensor_configs_json)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON for 'sensor_configs_json': {e}. Using empty config.")
            self.sensor_configs = []

        self.last_voltages = {} # Stores last received voltage for each channel
        self.publishers = {} # Stores publishers for interpreted sensor data

        # 3. Setup Subscriptions and Publishers based on config
        for config in self.sensor_configs:
            channel = config.get('channel')
            sensor_type = config.get('type')
            
            if channel is None or sensor_type is None:
                self.get_logger().warn(f"Skipping invalid sensor config: {config}")
                continue
            
            # Subscribe to the specific voltage topic from ADS1115
            self.create_subscription(
                Float32,
                f'/{self.adc_node_name}/voltage_ch{channel}',
                lambda msg, ch=channel, s_type=sensor_type: self._voltage_callback(ch, s_type, msg),
                10
            )
            self.get_logger().info(f"Subscribing to /voltage_ch{channel} for {sensor_type}.")

            # Create publishers for interpreted data
            if sensor_type == 'thermistor':
                self.publishers[channel] = self.create_publisher(Temperature, f'~/temperature_ch{channel}', 10)
            elif sensor_type == 'ldr' or sensor_type == 'temt6000' or sensor_type == 'guva_s12' or sensor_type == 'ml8511':
                self.publishers[channel] = self.create_publisher(Illuminance, f'~/illuminance_ch{channel}', 10) # Using illuminance for UV also
            elif sensor_type == 'acs712' or sensor_type == 'max471':
                # No standard Current msg in sensor_msgs. Using Float32 with unit in frame_id/message
                self.publishers[channel] = self.create_publisher(Float32, f'~/current_ch{channel}', 10)
            elif sensor_type == 'voltage_divider':
                self.publishers[channel] = self.create_publisher(Float32, f'~/voltage_divider_ch{channel}', 10)
            elif sensor_type == 'soil_moisture':
                self.publishers[channel] = self.create_publisher(Float32, f'~/soil_moisture_ch{channel}', 10)
            elif sensor_type == 'hr202':
                self.publishers[channel] = self.create_publisher(RelativeHumidity, f'~/humidity_ch{channel}', 10)
            elif sensor_type == 'vibration' or sensor_type == 'noise_level':
                self.publishers[channel] = self.create_publisher(Range, f'~/analog_range_ch{channel}', 10) # Using Range for generic 0-X level
            else:
                self.get_logger().warn(f"Unsupported sensor type '{sensor_type}' for channel {channel}. No publisher created.")

        self.timer = self.create_timer(1.0 / self.publish_rate, self._interpret_and_publish_timer)

    def _voltage_callback(self, channel: int, sensor_type: str, msg: Float32):
        """Stores the latest voltage reading for a channel."""
        self.last_voltages[channel] = msg.data

    def _interpret_and_publish_timer(self):
        """Periodically interprets stored voltages and publishes ROS2 messages."""
        current_time = self.get_clock().now().to_msg()
        for config in self.sensor_configs:
            channel = config.get('channel')
            sensor_type = config.get('type')
            
            voltage = self.last_voltages.get(channel)
            if voltage is None or channel not in self.publishers:
                continue

            pub = self.publishers[channel]
            
            try:
                if sensor_type == 'thermistor':
                    # Example: NTC Thermistor, requires R_ref, B_const, T_nominal, R_nominal
                    # Simplified Steinhart-Hart equation or lookup for demonstration
                    R_ref = config.get('R_ref', 10000.0) # Reference resistance at T_nominal
                    B_const = config.get('B_const', 3950.0) # Beta constant
                    T_nominal = config.get('T_nominal', 298.15) # Nominal temperature in Kelvin (25C)
                    
                    # Assume thermistor in voltage divider with R_ref to GND, and V_supply to thermistor
                    V_supply = config.get('V_supply', 3.3)
                    if voltage == 0:
                        resistance = float('inf')
                    else:
                        resistance = R_ref * (V_supply / voltage - 1)
                    
                    if resistance <= 0 or math.isinf(resistance) or math.isnan(resistance):
                         temperature_k = float('nan')
                    else:
                         temperature_k = 1.0 / ((1.0 / T_nominal) + (1.0 / B_const) * math.log(resistance / R_ref))
                    
                    temp_msg = Temperature()
                    temp_msg.header.stamp = current_time
                    temp_msg.header.frame_id = f'{self.frame_id}_ch{channel}'
                    temp_msg.temperature = temperature_k - 273.15 # Convert to Celsius
                    pub.publish(temp_msg)

                elif sensor_type == 'ldr':
                    # LDR in voltage divider: V_supply -> LDR -> ADC_IN -> R_ref -> GND
                    R_ref = config.get('R_ref', 10000.0)
                    V_supply = config.get('V_supply', 3.3)
                    
                    if voltage >= V_supply or voltage <= 0: # Avoid division by zero or invalid range
                         resistance_ldr = float('inf') if voltage <= 0 else 0.0
                    else:
                         resistance_ldr = R_ref * (V_supply / voltage - 1)
                    
                    # Simplified lux conversion (very rough, LDRs are non-linear)
                    # lux = 500 / (resistance_ldr / 1000) for common LDRs (rough approx)
                    lux = config.get('lux_factor', 500000.0) / resistance_ldr if resistance_ldr > 0 else 0.0

                    illum_msg = Illuminance()
                    illum_msg.header.stamp = current_time
                    illum_msg.header.frame_id = f'{self.frame_id}_ch{channel}'
                    illum_msg.illuminance = lux
                    pub.publish(illum_msg)

                elif sensor_type == 'temt6000' or sensor_type == 'guva_s12' or sensor_type == 'ml8511':
                    # Linear photo/UV sensors often output voltage proportional to light/UV intensity
                    # Convert voltage to some arbitrary intensity or a calibrated value (e.g., mW/cm^2 for UV)
                    # For TEMT6000, 1V could be ~1000 Lux. For UV, 1V could be ~10mW/cm^2
                    
                    # Simple linear conversion: output_unit = (voltage - offset) * scale
                    offset_v = config.get('offset_voltage', 0.0)
                    scale_factor = config.get('scale_factor', 1.0)
                    
                    interpreted_value = (voltage - offset_v) * scale_factor
                    
                    illum_msg = Illuminance()
                    illum_msg.header.stamp = current_time
                    illum_msg.header.frame_id = f'{self.frame_id}_ch{channel}'
                    illum_msg.illuminance = interpreted_value
                    # For UV, the unit is not lux, but Illuminance msg is the closest.
                    # A custom message for UV index or mW/cm^2 might be better long-term.
                    pub.publish(illum_msg)

                elif sensor_type == 'acs712' or sensor_type == 'max471':
                    # ACS712 outputs voltage proportional to current. At 0A, output is VCC/2.
                    # E.g., ACS712-05B: 100mV/A sensitivity, VCC=5V -> 2.5V at 0A
                    sensitivity = config.get('sensitivity_mv_per_a', 185) / 1000.0 # V/A
                    offset_v = config.get('offset_voltage', 2.5) # V
                    
                    current_a = (voltage - offset_v) / sensitivity
                    
                    current_msg = Float32()
                    current_msg.data = current_a
                    pub.publish(current_msg) # No sensor_msgs/Current type

                elif sensor_type == 'voltage_divider':
                    # Measure a higher voltage using a known voltage divider ratio
                    ratio = config.get('ratio', 2.0) # e.g., 2.0 means input voltage = ADC_voltage * 2
                    source_voltage = voltage * ratio
                    
                    voltage_msg = Float32()
                    voltage_msg.data = source_voltage
                    pub.publish(voltage_msg)

                                elif sensor_type == 'soil_moisture':
                                    # Capacitive soil moisture sensors output voltage inversely proportional to moisture
                                    min_v = config.get('min_voltage', 0.8) # Dryest
                                    max_v = config.get('max_voltage', 2.5) # Wettest
                                    
                                    moisture_percent = np.interp(voltage, [min_v, max_v], [0.0, 100.0])
                                    moisture_msg = Float32()
                                    moisture_msg.data = max(0.0, min(100.0, moisture_percent)) # Clamp 0-100
                                    pub.publish(moisture_msg)
                
                                elif sensor_type == 'hr202':
                                    # HR202 Resistive Humidity Sensor
                                    # Circuit: Vcc -> HR202 -> ADC_IN (voltage) -> R_ref -> GND
                                    R_ref = config.get('R_ref', 10000.0) 
                                    V_supply = config.get('V_supply', 3.3)
                                    
                                    if voltage >= V_supply or voltage <= 0.01:
                                        resistance = float('inf') if voltage <= 0.01 else 0.0
                                    else:
                                        # voltage = V_supply * R_ref / (Rs + R_ref)
                                        # Rs = R_ref * (V_supply - voltage) / voltage
                                        resistance = R_ref * (V_supply - voltage) / voltage
                                    
                                    # HR202 Characteristic is logarithmic. 
                                    # Approx: Log10(R) = A * Humidity + B  => Humidity = (Log10(R) - B) / A
                                    # These values are generic approximations for HR202 at 25C
                                    # Log10(R) at 20% RH ~= 6.5 (3M Ohm), at 90% RH ~= 3.5 (3k Ohm)
                                    if resistance <= 0 or math.isinf(resistance):
                                        humidity_pct = 0.0
                                    else:
                                        log_r = math.log10(resistance)
                                        # Linear interpolation between common points (log scale)
                                        # 20% -> 6.5, 40% -> 5.5, 60% -> 4.5, 80% -> 3.8, 90% -> 3.5
                                        rh_points = [20.0, 40.0, 60.0, 80.0, 90.0]
                                        log_r_points = [6.5, 5.5, 4.5, 3.8, 3.5]
                                        humidity_pct = np.interp(log_r, log_r_points[::-1], rh_points[::-1])
                                    
                                    hum_msg = RelativeHumidity()
                                    hum_msg.header.stamp = current_time
                                    hum_msg.header.frame_id = f'{self.frame_id}_ch{channel}'
                                    hum_msg.relative_humidity = float(max(0.0, min(100.0, humidity_pct))) / 100.0
                                    pub.publish(hum_msg)
                                
                                elif sensor_type == 'vibration' or sensor_type == 'noise_level':                                # Generic analog level, use sensor_msgs/Range (min/max range can be voltage levels)
                                # Or a custom message for 'level'
                                # For now, just publish raw voltage
                                range_msg = Range()
                                range_msg.header.stamp = current_time
                                range_msg.header.frame_id = f'{self.frame_id}_ch{channel}'
                                range_msg.range = voltage
                                range_msg.min_range = config.get('min_voltage', 0.0)
                                range_msg.max_range = config.get('max_voltage', 3.3)
                                pub.publish(range_msg)
                            
                            elif sensor_type == 'mq_gas':
                                # MQ-series gas sensors need calibration: R0 (resistance in clean air), Rs/R0 vs PPM curve
                                # Assume a voltage divider (Vcc - RL - sensor_heater - sensor_resistance - GND)
                                # And the voltage is read across the sensor_resistance (between sensor and GND)
                                
                                # First, get sensor resistance (Rs) from measured voltage
                                V_supply = config.get('V_supply', 5.0) # MQ sensors often use 5V
                                RL = config.get('RL', 10000.0) # Load resistor value
                                if voltage >= V_supply or voltage <= 0:
                                    Rs = float('inf') if voltage <= 0 else 0.0
                                else:
                                    Rs = RL * (V_supply - voltage) / voltage
                                
                                R0 = config.get('R0', 100000.0) # R0 from calibration in clean air
                                gas_type = config.get('gas_type', 'unknown')
                                
                                if Rs == 0 or R0 == 0:
                                    ppm = float('nan')
                                else:
                                    Rs_R0 = Rs / R0
                                    # This is a very simplified example. Real calibration uses curves (e.g., log-log plot).
                                    # These parameters (A, B) come from linearizing the log-log plot (e.g., for MQ-2 for LPG)
                                    # PPM = A * (Rs/R0)^B
                                    A = config.get('A_calib', 10000.0) # A for PPM = A * (Rs/R0)^B
                                    B = config.get('B_calib', -2.5) # B for PPM = A * (Rs/R0)^B
                                    ppm = A * (Rs_R0**B)
                                
                                # Publish as Float32, with sensor type and unit in frame_id or metadata
                                gas_msg = Float32()
                                gas_msg.data = ppm
                                # Consider adding custom message type for gas concentrations in future
                                pub.publish(gas_msg)
                
                            except Exception as e:
                                self.get_logger().error(f"Error interpreting channel {channel} ({sensor_type}): {e}")
def main(args=None):
    rclpy.init(args=args)
    node = AnalogSensorInterpreterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
