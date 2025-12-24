import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import String
import time
import math

# Try to import the hardware library
try:
    import board
    import adafruit_bno055
    HAS_HARDWARE = True
except ImportError:
    HAS_HARDWARE = False

class BNO055Node(Node):
    """
    ROS2 Node for the BNO055 Intelligent IMU.
    Publishes orientation (Quaternions/Euler), angular velocity, and linear acceleration.
    """

    def __init__(self):
        super().__init__('bno055_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('publish_rate', 20.0) # Hz
        self.declare_parameter('frame_id', 'bno055_link')
        self.declare_parameter('mock_hardware', not HAS_HARDWARE)

        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.mock_mode = self.get_parameter('mock_hardware').value

        # Hardware Setup
        if self.mock_mode:
            self.get_logger().warn('BNO055: Running in MOCK mode.')
            self.mock_time = time.monotonic()
        else:
            if not HAS_HARDWARE:
                self.get_logger().error('BNO055 libraries not found. Install with: pip install adafruit-circuitpython-bno055')
                raise RuntimeError('Library missing')
            
            try:
                # Initialize I2C and sensor
                i2c = board.I2C()
                self.sensor = adafruit_bno055.BNO055_I2C(i2c)
                self.get_logger().info('BNO055 initialized successfully.')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize BNO055 hardware: {e}')
                self.mock_mode = True

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '~/imu', 10)
        self.euler_pub = self.create_publisher(Vector3, '~/euler', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def timer_callback(self):
        try:
            if self.mock_mode:
                # Simulate some drift and orientation
                t = time.monotonic() - self.mock_time
                roll = 5.0 * math.sin(t)
                pitch = 3.0 * math.cos(t * 0.5)
                yaw = (t * 10.0) % 360.0
                quat = (0.0, 0.0, 0.0, 1.0)
                gyro = (0.0, 0.0, 0.0)
                accel = (0.0, 0.0, 9.81)
                sys, gyro_cal, accel_cal, mag_cal = (3, 3, 3, 3)
            else:
                # Read hardware
                quat = self.sensor.quaternion # x, y, z, w
                euler = self.sensor.euler # yaw, roll, pitch
                gyro = self.sensor.gyro
                accel = self.sensor.linear_acceleration
                sys, gyro_cal, accel_cal, mag_cal = self.sensor.calibration_status
                
                # BNO055 euler is (yaw, roll, pitch)
                yaw, roll, pitch = euler if euler[0] is not None else (0.0, 0.0, 0.0)

            # 1. Publish Imu Message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            if quat and quat[0] is not None:
                imu_msg.orientation.x = float(quat[0])
                imu_msg.orientation.y = float(quat[1])
                imu_msg.orientation.z = float(quat[2])
                imu_msg.orientation.w = float(quat[3])
            
            if gyro and gyro[0] is not None:
                imu_msg.angular_velocity.x = float(gyro[0])
                imu_msg.angular_velocity.y = float(gyro[1])
                imu_msg.angular_velocity.z = float(gyro[2])
            
            if accel and accel[0] is not None:
                imu_msg.linear_acceleration.x = float(accel[0])
                imu_msg.linear_acceleration.y = float(accel[1])
                imu_msg.linear_acceleration.z = float(accel[2])

            self.imu_pub.publish(imu_msg)

            # 2. Publish Euler (Degrees)
            euler_msg = Vector3()
            euler_msg.x = float(roll)
            euler_msg.y = float(pitch)
            euler_msg.z = float(yaw)
            self.euler_pub.publish(euler_msg)

            # 3. Publish Status
            status_msg = String()
            status_msg.data = f"Sys:{sys} G:{gyro_cal} A:{accel_cal} M:{mag_cal}"
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error reading BNO055: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
