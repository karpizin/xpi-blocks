import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Temperature, Illuminance, FluidPressure, Range, RelativeHumidity
import json
import math
import numpy as np

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
            elif sensor_type == 'ldr':
                self.publishers[channel] = self.create_publisher(Illuminance, f'~/illuminance_ch{channel}', 10)
            elif sensor_type == 'temt6000' or sensor_type == 'guva_s12' or sensor_type == 'ml8511':
                self.publishers[channel] = self.create_publisher(Illuminance, f'~/illuminance_ch{channel}', 10)
            elif sensor_type == 'acs712' or sensor_type == 'max471':
                self.publishers[channel] = self.create_publisher(Float32, f'~/current_ch{channel}', 10)
            elif sensor_type == 'voltage_divider':
                self.publishers[channel] = self.create_publisher(Float32, f'~/voltage_divider_ch{channel}', 10)
            elif sensor_type == 'soil_moisture':
                self.publishers[channel] = self.create_publisher(Float32, f'~/soil_moisture_ch{channel}', 10)
            elif sensor_type == 'hr202':
                self.publishers[channel] = self.create_publisher(RelativeHumidity, f'~/humidity_ch{channel}', 10)
            elif sensor_type == 'vibration' or sensor_type == 'noise_level':
                self.publishers[channel] = self.create_publisher(Range, f'~/analog_range_ch{channel}', 10)
            elif sensor_type == 'mq_gas':
                self.publishers[channel] = self.create_publisher(Float32, f'~/gas_ch{channel}', 10)
            else:
                self.get_logger().warn(f"Unsupported sensor type '{sensor_type}' for channel {channel}. No publisher created.")

        self.timer = self.create_timer(1.0 / self.publish_rate, self._interpret_and_publish_timer)

    def _voltage_callback(self, channel, sensor_type, msg):
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
                    # NTC Thermistor
                    R_ref = config.get('R_ref', 10000.0)
                    B_const = config.get('B_const', 3950.0)
                    T_nominal = config.get('T_nominal', 298.15)
                    V_supply = config.get('V_supply', 3.3)
                    
                    if voltage <= 0:
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
                    temp_msg.temperature = temperature_k - 273.15
                    pub.publish(temp_msg)

                elif sensor_type == 'ldr':
                    # LDR Light Sensor
                    R_ref = config.get('R_ref', 10000.0)
                    V_supply = config.get('V_supply', 3.3)
                    
                    if voltage >= V_supply or voltage <= 0:
                         resistance_ldr = float('inf') if voltage <= 0 else 0.0
                    else:
                         resistance_ldr = R_ref * (V_supply / voltage - 1)
                    
                    lux = config.get('lux_factor', 500000.0) / resistance_ldr if resistance_ldr > 0 else 0.0
                    illum_msg = Illuminance()
                    illum_msg.header.stamp = current_time
                    illum_msg.header.frame_id = f'{self.frame_id}_ch{channel}'
                    illum_msg.illuminance = lux
                    pub.publish(illum_msg)

                elif sensor_type == 'temt6000' or sensor_type == 'guva_s12' or sensor_type == 'ml8511':
                    # Linear Light/UV Sensors
                    offset_v = config.get('offset_voltage', 0.0)
                    scale_factor = config.get('scale_factor', 1.0)
                    
                    interpreted_value = (voltage - offset_v) * scale_factor
                    
                    illum_msg = Illuminance()
                    illum_msg.header.stamp = current_time
                    illum_msg.header.frame_id = f'{self.frame_id}_ch{channel}'
                    illum_msg.illuminance = interpreted_value
                    pub.publish(illum_msg)

                elif sensor_type == 'acs712' or sensor_type == 'max471':
                    # Current Sensors
                    sensitivity = config.get('sensitivity_mv_per_a', 185) / 1000.0
                    offset_v = config.get('offset_voltage', 2.5)
                    
                    current_a = (voltage - offset_v) / sensitivity
                    pub.publish(Float32(data=current_a))

                elif sensor_type == 'voltage_divider':
                    # Simple Voltage Divider
                    ratio = config.get('ratio', 2.0)
                    source_voltage = voltage * ratio
                    pub.publish(Float32(data=source_voltage))

                elif sensor_type == 'soil_moisture':
                    # Capacitive Soil Moisture
                    min_v = config.get('min_voltage', 0.8)
                    max_v = config.get('max_voltage', 2.5)
                    
                    moisture_percent = np.interp(voltage, [min_v, max_v], [0.0, 100.0])
                    # Ensure 0-100 clamp
                    moisture_percent = max(0.0, min(100.0, moisture_percent))
                    pub.publish(Float32(data=moisture_percent))

                elif sensor_type == 'hr202':
                    # HR202 Humidity Sensor
                    R_ref = config.get('R_ref', 10000.0) 
                    V_supply = config.get('V_supply', 3.3)
                    
                    if voltage >= V_supply or voltage <= 0.01:
                        resistance = float('inf') if voltage <= 0.01 else 0.0
                    else:
                        resistance = R_ref * (V_supply - voltage) / voltage
                    
                    if resistance <= 0 or math.isinf(resistance):
                        humidity_pct = 0.0
                    else:
                        # Logarithmic approximation
                        log_r = math.log10(resistance)
                        rh_points = [20.0, 40.0, 60.0, 80.0, 90.0]
                        log_r_points = [6.5, 5.5, 4.5, 3.8, 3.5]
                        humidity_pct = np.interp(log_r, log_r_points[::-1], rh_points[::-1])
                    
                    humidity_pct = max(0.0, min(100.0, humidity_pct))
                    
                    hum_msg = RelativeHumidity()
                    hum_msg.header.stamp = current_time
                    hum_msg.header.frame_id = f'{self.frame_id}_ch{channel}'
                    hum_msg.relative_humidity = float(humidity_pct) / 100.0
                    pub.publish(hum_msg)

                elif sensor_type == 'vibration' or sensor_type == 'noise_level':
                    # Generic Range Sensor
                    range_msg = Range()
                    range_msg.header.stamp = current_time
                    range_msg.header.frame_id = f'{self.frame_id}_ch{channel}'
                    range_msg.range = voltage
                    range_msg.min_range = config.get('min_voltage', 0.0)
                    range_msg.max_range = config.get('max_voltage', 3.3)
                    pub.publish(range_msg)
                
                elif sensor_type == 'mq_gas':
                    # MQ Gas Sensors
                    V_supply = config.get('V_supply', 5.0)
                    RL = config.get('RL', 10000.0)
                    
                    if voltage >= V_supply or voltage <= 0:
                        Rs = float('inf') if voltage <= 0 else 0.0
                    else:
                        Rs = RL * (V_supply - voltage) / voltage
                    
                    R0 = config.get('R0', 100000.0)
                    
                    if Rs == 0 or R0 == 0:
                        ppm = float('nan')
                    else:
                        Rs_R0 = Rs / R0
                        A = config.get('A_calib', 10000.0)
                        B = config.get('B_calib', -2.5)
                        ppm = A * (Rs_R0**B)
                    
                    pub.publish(Float32(data=float(ppm)))

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
