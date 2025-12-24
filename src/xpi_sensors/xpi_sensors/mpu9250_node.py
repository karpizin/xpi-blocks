import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
import time
import math

# Try to import the hardware library
try:
    from mpu9250_jmdev.registers import *
    from mpu9250_jmdev.mpu_9250 import MPU9250
    HAS_HARDWARE = True
except ImportError:
    HAS_HARDWARE = False

class MPU9250Node(Node):
    """
    ROS2 Node for the MPU9250 9-DOF IMU.
    Publishes acceleration, angular velocity, and magnetic field data.
    """

    def __init__(self):
        super().__init__('mpu9250_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('publish_rate', 20.0) # Hz
        self.declare_parameter('frame_id', 'mpu9250_link')
        self.declare_parameter('mock_hardware', not HAS_HARDWARE)

        self.rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        # Hardware Setup
        if self.mock_mode:
            self.get_logger().warn('MPU9250: Running in MOCK mode.')
            self.mock_time = time.monotonic()
        else:
            if not HAS_HARDWARE:
                self.get_logger().error('MPU9250 library not found. Install with: pip install mpu9250-jmdev')
                raise RuntimeError('Library missing')
            
            try:
                # Initialize using the library
                self.mpu = MPU9250(
                    address_ak=AK8963_ADDRESS, 
                    address_mpu_master=MPU9050_ADDRESS_68, # 0x68
                    bus=self.get_parameter('i2c_bus').value, 
                    debug=False
                )
                self.mpu.configure()
                self.get_logger().info('MPU9250 initialized successfully.')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize MPU9250 hardware: {e}')
                self.mock_mode = True

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '~/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, '~/mag', 10)
        self.temp_pub = self.create_publisher(Temperature, '~/temp', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def timer_callback(self):
        try:
            if self.mock_mode:
                # Simulate some movement
                t = time.monotonic() - self.mock_time
                accel = [0.0, 0.0, 9.81]
                gyro = [0.1 * math.sin(t), 0.0, 0.0]
                mag = [20.0, 30.0, -10.0]
                temp = 25.0 + math.sin(t)
            else:
                # Read hardware
                accel = self.mpu.readAccelerometerMaster() # [x, y, z] in m/s^2? No, g usually.
                gyro = self.mpu.readGyroscopeMaster() # [x, y, z] in dps
                mag = self.mpu.readMagnetometerMaster() # [x, y, z] in uT
                temp = self.mpu.readTemperatureMaster()

            current_time = self.get_clock().now().to_msg()

            # 1. IMU Message
            imu_msg = Imu()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = self.frame_id
            
            # Linear Acceleration (convert g to m/s^2 if needed, jmdev returns g)
            imu_msg.linear_acceleration.x = float(accel[0]) * 9.80665
            imu_msg.linear_acceleration.y = float(accel[1]) * 9.80665
            imu_msg.linear_acceleration.z = float(accel[2]) * 9.80665
            
            # Angular Velocity (convert dps to rad/s)
            imu_msg.angular_velocity.x = math.radians(float(gyro[0]))
            imu_msg.angular_velocity.y = math.radians(float(gyro[1]))
            imu_msg.angular_velocity.z = math.radians(float(gyro[2]))
            
            # No orientation estimation in this simple driver (published as identity)
            imu_msg.orientation.w = 1.0
            
            self.imu_pub.publish(imu_msg)

            # 2. Magnetic Field Message (convert uT to Tesla)
            mag_msg = MagneticField()
            mag_msg.header.stamp = current_time
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = float(mag[0]) * 1e-6
            mag_msg.magnetic_field.y = float(mag[1]) * 1e-6
            mag_msg.magnetic_field.z = float(mag[2]) * 1e-6
            self.mag_pub.publish(mag_msg)

            # 3. Temperature
            temp_msg = Temperature()
            temp_msg.header.stamp = current_time
            temp_msg.header.frame_id = self.frame_id
            temp_msg.temperature = float(temp)
            self.temp_pub.publish(temp_msg)

        except Exception as e:
            self.get_logger().error(f'Error reading MPU9250: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MPU9250Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
