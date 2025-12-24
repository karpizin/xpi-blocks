import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from std_srvs.srv import Trigger
from gpiozero import DigitalInputDevice, DigitalOutputDevice, Device
from gpiozero.pins.mock import MockFactory
import time
import threading

class HX711Node(Node):
    """
    ROS2 Node for HX711 24-bit ADC.
    Interfaces with load cells to measure weight/force.
    """

    def __init__(self):
        super().__init__('hx711_node')

        # Parameters
        self.declare_parameter('data_pin', 5)
        self.declare_parameter('clock_pin', 6)
        self.declare_parameter('reference_unit', 1.0) # Divider to convert raw to grams/kg
        self.declare_parameter('publish_rate', 5.0) # Hz
        self.declare_parameter('gain', 128) # 128, 64 or 32
        self.declare_parameter('mock_hardware', False)

        self.data_p = self.get_parameter('data_pin').value
        self.clock_p = self.get_parameter('clock_pin').value
        self.ref_unit = self.get_parameter('reference_unit').value
        self.rate = self.get_parameter('publish_rate').value
        self.gain = self.get_parameter('gain').value
        mock_mode = self.get_parameter('mock_hardware').value

        if mock_mode:
            Device.pin_factory = MockFactory()
            self.get_logger().warn('HX711: Running in MOCK mode.')

        # GPIO Setup
        self.pd_sck = DigitalOutputDevice(self.clock_p)
        self.dout = DigitalInputDevice(self.data_p)

        # State
        self.offset = 0
        self.last_raw = 0
        
        # Publishers
        self.weight_pub = self.create_publisher(Float32, '~/weight', 10)
        self.raw_pub = self.create_publisher(Int32, '~/raw', 10)

        # Services
        self.tare_srv = self.create_service(Trigger, '~/tare', self.tare_callback)

        # Initial Tare
        self.get_logger().info("HX711 initialized. Performing initial tare...")
        self.tare()

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def tare_callback(self, request, response):
        self.tare()
        response.success = True
        response.message = f"Tared successfully. New offset: {self.offset}"
        return response

    def tare(self, times=15):
        """Zeroes out the scale."""
        sum_raw = 0
        for _ in range(times):
            sum_raw += self.read_raw()
            time.sleep(0.01)
        self.offset = sum_raw / times
        self.get_logger().info(f"Tare complete. Offset: {self.offset}")

    def read_raw(self):
        """Reads 24-bit data from HX711."""
        if Device.pin_factory.__class__.__name__ == 'MockFactory':
            return int(50000 + 1000 * time.time() % 100)

        # Wait for data to be ready (DOUT goes LOW)
        timeout = 0.5
        start = time.time()
        while self.dout.value:
            if time.time() - start > timeout:
                self.get_logger().error("HX711 timeout: Data not ready")
                return self.last_raw
        
        raw_data = 0
        for _ in range(24):
            self.pd_sck.on()
            raw_data = (raw_data << 1) | self.dout.value
            self.pd_sck.off()

        # Set gain for next conversion (1, 2 or 3 pulses)
        # 1 pulse: Gain 128 (Channel A)
        # 2 pulses: Gain 32 (Channel B)
        # 3 pulses: Gain 64 (Channel A)
        pulses = 1
        if self.gain == 32: pulses = 2
        elif self.gain == 64: pulses = 3
        
        for _ in range(pulses):
            self.pd_sck.on()
            self.pd_sck.off()

        # Handle 2's complement
        if raw_data & 0x800000:
            raw_data -= 0x1000000

        self.last_raw = raw_data
        return raw_data

    def timer_callback(self):
        raw = self.read_raw()
        
        # Publish Raw
        raw_msg = Int32()
        raw_msg.data = int(raw)
        self.raw_pub.publish(raw_msg)

        # Publish Weight
        weight_msg = Float32()
        weight_msg.data = float((raw - self.offset) / self.ref_unit)
        self.weight_pub.publish(weight_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HX711Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
