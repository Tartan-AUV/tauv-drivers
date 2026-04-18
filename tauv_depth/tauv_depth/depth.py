import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from . import ms5837


class Bar02Node(Node):
    def __init__(self):
        super().__init__('bar02_node')

        self.bus_id = self.declare_parameter('i2c_bus', 7).value
        self.freq = self.declare_parameter('freq_hz', 10.0).value
        self.max_retries = self.declare_parameter('max_retries', 5).value
        self.variance = self.declare_parameter('variance', 0.001).value

        self.sensor = None
        self.error_count = 0

        self.initialize_sensor()

        self.publisher = self.create_publisher(Odometry, '/os/sensors/depth', 10)
        self.timer = self.create_timer(1.0 / self.freq, self.timer_callback)
        self.get_logger().info(f'Bar02 node started on I2C bus {self.bus_id}')

    def initialize_sensor(self):
        try:
            self.sensor = ms5837.MS5837_02BA(bus=self.bus_id)
            if not self.sensor.init():
                self.get_logger().error('Sensor init failed (check wiring/power)')
                self.sensor = None
                return
            self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
            self.error_count = 0
        except Exception as e:
            self.get_logger().error(f'I2C bus {self.bus_id} unavailable: {e}')
            self.sensor = None

    def timer_callback(self):
        if self.sensor is None:
            self.initialize_sensor()
            return

        try:
            if not self.sensor.read():
                raise OSError('read() returned False')
        except Exception as e:
            self.error_count += 1
            self.get_logger().warn(
                f'Bar02 read failed ({self.error_count}/{self.max_retries}): {e}',
                throttle_duration_sec=2.0,
            )
            if self.error_count >= self.max_retries:
                self.get_logger().error('Too many failures, re-initializing sensor.')
                self.sensor = None
            return

        depth = float(self.sensor.depth())

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'depth_link'

        # Convert depth (positive down) to ROS Z (positive up)
        odom.pose.pose.position.z = -depth

        # High covariance on everything except Z (index 14 = z,z in row-major 6x6)
        odom.pose.covariance = [1e6] * 36
        odom.twist.covariance = [1e6] * 36
        odom.pose.covariance[14] = self.variance

        self.publisher.publish(odom)

        if self.error_count > 0:
            self.get_logger().info('Bar02 recovered.')
            self.error_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = Bar02Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()