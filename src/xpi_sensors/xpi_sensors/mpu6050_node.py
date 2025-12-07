import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature
from geometry_msgs.msg import Quaternion, Vector3
from xpi_commons.i2c_helper import get_smbus
import time
import struct
import math

class MPU6050Node(Node):
    """
    ROS2 Node for MPU6050 6-DOF IMU sensor.
    Reads accelerometer, gyroscope, and temperature via I2C.
    Publishes sensor_msgs/Imu and sensor_msgs/Temperature.
    """
    # MPU6050 Registers
    PWR_MGMT_1   = 0x6B
    SMPLRT_DIV   = 0x19
    CONFIG       = 0x1A
    GYRO_CONFIG  = 0x1B
    INT_ENABLE   = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    TEMP_OUT_H   = 0x41
    GYRO_XOUT_H  = 0x43
    GYRO_YOUT_H  = 0x45
    GYRO_ZOUT_H  = 0x47

    # Sample Rate Divider. 1000Hz / (1 + SMPLRT_DIV)
    # So if SMPLRT_DIV = 7, sample rate = 125Hz.
    # We will use this to set desired read rate.
    
    # Gyro Full Scale Range (GYRO_CONFIG)
    # 0 = +/- 250 deg/s
    # 1 = +/- 500 deg/s
    # 2 = +/- 1000 deg/s
    # 3 = +/- 2000 deg/s

    # Accel Full Scale Range (ACCEL_CONFIG) - part of CONFIG register
    # 0 = +/- 2g
    # 1 = +/- 4g
    # 2 = +/- 8g
    # 3 = +/- 16g

    def __init__(self):
        super().__init__('mpu6050_node')

        # 1. Declare Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('publish_rate', 50.0) # Hz
        self.declare_parameter('accel_fsr', 0) # Accel Full Scale Range (0=2g, 1=4g, 2=8g, 3=16g)
        self.declare_parameter('gyro_fsr', 0)  # Gyro Full Scale Range (0=250dps, 1=500dps, 2=1000dps, 3=2000dps)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('mock_hardware', False)

        # 2. Read Parameters
        self.bus_id = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.accel_fsr_val = self.get_parameter('accel_fsr').value
        self.gyro_fsr_val = self.get_parameter('gyro_fsr').value
        self.frame_id = self.get_parameter('frame_id').value
        mock_mode = self.get_parameter('mock_hardware').value

        # 3. Init I2C
        self.bus = get_smbus(self.bus_id, mock=mock_mode)
        
        # Scale factors (per LSB)
        self.accel_scale = self._get_accel_scale(self.accel_fsr_val)
        self.gyro_scale = self._get_gyro_scale(self.gyro_fsr_val)

        try:
            self.init_mpu6050()
            self.get_logger().info(f'MPU6050 initialized at 0x{self.address:02X} on bus {self.bus_id}.')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MPU6050: {e}. Falling back to mock.')
            self.bus.close() # Ensure mock bus is used if real fails
            self.bus = get_smbus(self.bus_id, mock=True) # Re-init as mock
            mock_mode = True

        if mock_mode:
            self.get_logger().warn('MPU6050: Running in MOCK mode. No real I2C data will be read.')
            self.mock_accel = [0.1, 0.0, 9.81]
            self.mock_gyro = [0.01, 0.02, 0.03]
            self.mock_temp = 25.0
            self.mock_counter = 0

        # 4. Publishers
        self.imu_publisher = self.create_publisher(Imu, '~/imu/data_raw', 10)
        self.temp_publisher = self.create_publisher(Temperature, '~/temperature', 10)
        
        # 5. Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info(f'MPU6050: Publishing IMU and Temperature at {self.publish_rate} Hz.')

    def _get_accel_scale(self, fsr):
        if fsr == 0: return 16384.0 # +/- 2g
        if fsr == 1: return 8192.0  # +/- 4g
        if fsr == 2: return 4096.0  # +/- 8g
        if fsr == 3: return 2048.0  # +/- 16g
        self.get_logger().warn(f"Invalid Accel FSR: {fsr}. Defaulting to 2g.")
        return 16384.0

    def _get_gyro_scale(self, fsr):
        if fsr == 0: return 131.0 # +/- 250 dps
        if fsr == 1: return 65.5  # +/- 500 dps
        if fsr == 2: return 32.8  # +/- 1000 dps
        if fsr == 3: return 16.4  # +/- 2000 dps
        self.get_logger().warn(f"Invalid Gyro FSR: {fsr}. Defaulting to 250 dps.")
        return 131.0

    def init_mpu6050(self):
        # Wake up MPU6050
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        time.sleep(0.1) # Allow to power up
        
        # Set sample rate divider (e.g., to 7 for 125Hz if internal is 1kHz)
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, int(1000 / self.publish_rate) - 1)

        # Set Accelerometer Full Scale Range
        # Read CONFIG register and mask out existing ACCEL_CONFIG bits, then set new
        config_reg = self.bus.read_byte_data(self.address, self.CONFIG)
        config_reg &= 0b11100111 # Clear ACCEL_FS_SEL bits (bit 3 & 4)
        config_reg |= (self.accel_fsr_val << 3)
        self.bus.write_byte_data(self.address, self.CONFIG, config_reg)

        # Set Gyroscope Full Scale Range
        gyro_config_reg = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)
        gyro_config_reg &= 0b11100111 # Clear GYRO_FS_SEL bits (bit 3 & 4)
        gyro_config_reg |= (self.gyro_fsr_val << 3)
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_config_reg)

        # Enable data ready interrupt (optional, but good practice)
        self.bus.write_byte_data(self.address, self.INT_ENABLE, 0x01)
        self.get_logger().info("MPU6050 configured.")

    def read_word_2c(self, reg):
        """Reads two bytes and converts them to signed 16-bit integer."""
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000: # Two's complement conversion
            return -((65535 - val) + 1)
        else:
            return val

    def timer_callback(self):
        imu_msg = Imu()
        temp_msg = Temperature()
        
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        temp_msg.header.stamp = imu_msg.header.stamp
        temp_msg.header.frame_id = self.frame_id

        if self.bus.mock_mode: # Assuming mock_mode is an attribute of mock bus
            self.mock_counter += 1
            # Simulate some motion
            imu_msg.linear_acceleration.x = self.mock_accel[0] * math.sin(self.mock_counter * 0.05) + 0.1
            imu_msg.linear_acceleration.y = self.mock_accel[1]
            imu_msg.linear_acceleration.z = self.mock_accel[2] + math.cos(self.mock_counter * 0.05) * 0.5

            imu_msg.angular_velocity.x = self.mock_gyro[0] * math.cos(self.mock_counter * 0.03)
            imu_msg.angular_velocity.y = self.mock_gyro[1]
            imu_msg.angular_velocity.z = self.mock_gyro[2] * math.sin(self.mock_counter * 0.07)

            temp_msg.temperature = self.mock_temp + math.sin(self.mock_counter * 0.1) * 2.0
        else:
            try:
                # Read Accelerometer data
                accel_x = self.read_word_2c(self.ACCEL_XOUT_H) / self.accel_scale * 9.80665 # Convert to m/s^2
                accel_y = self.read_word_2c(self.ACCEL_YOUT_H) / self.accel_scale * 9.80665
                accel_z = self.read_word_2c(self.ACCEL_ZOUT_H) / self.accel_scale * 9.80665

                # Read Gyroscope data
                gyro_x = self.read_word_2c(self.GYRO_XOUT_H) / self.gyro_scale * (math.pi / 180.0) # Convert to rad/s
                gyro_y = self.read_word_2c(self.GYRO_YOUT_H) / self.gyro_scale * (math.pi / 180.0)
                gyro_z = self.read_word_2c(self.GYRO_ZOUT_H) / self.gyro_scale * (math.pi / 180.0)

                # Read Temperature data
                raw_temp = self.read_word_2c(self.TEMP_OUT_H)
                temperature_c = (raw_temp / 340.0) + 36.53 # MPU6050 datasheet conversion

                imu_msg.linear_acceleration.x = accel_x
                imu_msg.linear_acceleration.y = accel_y
                imu_msg.linear_acceleration.z = accel_z
                
                imu_msg.angular_velocity.x = gyro_x
                imu_msg.angular_velocity.y = gyro_y
                imu_msg.angular_velocity.z = gyro_z

                temp_msg.temperature = temperature_c
                temp_msg.variance = 0.0 # Placeholder
                imu_msg.orientation_covariance[0] = -1.0 # Indicate orientation is not set

            except Exception as e:
                self.get_logger().error(f'MPU6050: Error reading sensor data: {e}')
                return # Skip publishing if error

        self.imu_publisher.publish(imu_msg)
        self.temp_publisher.publish(temp_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus.close() # Close I2C bus
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
