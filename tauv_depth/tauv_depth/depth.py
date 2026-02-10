import rclpy
from rclpy.node import Node
from tauv_msgs.msg import Depth  # Import your custom message
from . import ms5837

class Bar02Node(Node):
    def __init__(self):
        super().__init__('bar02_node')
        
        # Defaulting to 7 as per your configuration
        self.declare_parameter('i2c_bus', 7)
        bus_id = self.get_parameter('i2c_bus').value

        try:
            self.sensor = ms5837.MS5837_02BA(bus=bus_id)
            if not self.sensor.init():
                self.get_logger().error("Sensor Bar02 could not be initialized")
                return
        except Exception as e:
            self.get_logger().error(f"Failed to connect to I2C bus {bus_id}: {e}")
            return

        # Using freshwater density (approx 997 kg/m^3)
        self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)

        # Update publisher to use Depth msg and a more descriptive topic name
        self.publisher_ = self.create_publisher(Depth, 'vehicle/depth', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) 
        self.get_logger().info(f"Bar02 Node Started on I2C bus {bus_id}")

    def timer_callback(self):
        if self.sensor.read():
            msg = Depth()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'bar02'
            msg.depth = float(self.sensor.depth())
            msg.pressure = float(self.sensor.pressure(ms5837.UNITS_mbar) * 100.0)
            msg.temperature = float(self.sensor.temperature())
            msg.variance = 0.001 
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Failed to read Bar02 sensor")

def main(args=None):
    rclpy.init(args=args)
    node = Bar02Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()