import rclpy
from rclpy.node import Node
from tauv_msgs.msg import Depth
from . import ms5837
import time

class Bar02Node(Node):
    def __init__(self):
        super().__init__('bar02_node')
        
        self.declare_parameter('i2c_bus', 7)
        self.bus_id = self.get_parameter('i2c_bus').value
        self.declare_parameter('freqHz', 10)
        self.freq = self.get_parameter('freqHz').value
        # Error tracking
        self.error_count = 0
        self.max_retries = 5
        self.sensor = None
        
        # Initial Connection
        self.initialize_sensor()

        self.publisher_ = self.create_publisher(Depth, 'vehicle/depth', 10)
        self.timer = self.create_timer( (1.0/self.freq), self.timer_callback) 
        self.get_logger().info(f"Bar02 Node Started on I2C bus {self.bus_id}")

    def initialize_sensor(self):
        try:
            self.sensor = ms5837.MS5837_02BA(bus=self.bus_id)
            if not self.sensor.init():
                self.get_logger().error("Hardware initialization failed (check wiring/power)")
                return False
            
            self.sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
            self.error_count = 0
            return True
        except Exception as e:
            self.get_logger().error(f"I2C Bus {self.bus_id} unavailable: {e}")
            return False

    def timer_callback(self):
        try:
            # Attempt to read from the sensor
            if self.sensor and self.sensor.read():
                msg = Depth()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'bar02'
                msg.depth = float(self.sensor.depth())
                msg.pressure = float(self.sensor.pressure(ms5837.UNITS_mbar) * 100.0)
                msg.temperature = float(self.sensor.temperature())
                msg.variance = 0.001 
                
                self.publisher_.publish(msg)
                
                # Success: Reset error counter if it was incremented
                if self.error_count > 0:
                    self.get_logger().info("Bar02 connection restored.")
                    self.error_count = 0
            else:
                raise OSError("Sensor read() returned False")

        except (OSError, Exception) as e:
            self.error_count += 1
            # Throttled logging to prevent console spam
            self.get_logger().warn(
                f"Bar02 Read Failed ({self.error_count}/{self.max_retries}): {e}", 
                throttle_duration_sec=2.0
            )

            if self.error_count >= self.max_retries:
                self.get_logger().error("Critical I2C failure. Attempting hardware reset...")
                time.sleep(0.1) # Brief settle time
                self.initialize_sensor()

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