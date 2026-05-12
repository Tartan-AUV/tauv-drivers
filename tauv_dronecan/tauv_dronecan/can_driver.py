#!/usr/bin/env python3

"""
TAKES IN /thruster_forces (tauv_msgs/ThrusterSetpoint) and sends appropriate commands to ESCs over DroneCAN
the mapping is based on this blue robotics image, the rest is dealt with internally https://discuss.bluerobotics.com/t/custom-thruster-configuration/4384

uses force_to_gain.py to convert from forces to ESC gain values, which are then sent as raw commands to the ESCs.

"""




import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,String
import dronecan
import time
import numpy as np
from tauv_dronecan.rpm_to_gain import rpm_to_gain
from tauv_dronecan.mapping import mapping

from tauv_msgs.msg import ThrusterSetpoint,EscTelemetry


class CANDriver(Node):
    def __init__(self):
        super().__init__('CANDriver')
        
        # Parameters
        self.declare_parameter('interface', 'can1')
        self.declare_parameter('node_id', 12)
        self.declare_parameter('bitrate', 1000000)
        self.declare_parameter('esc_count', 8)
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('discovery_time_sec', 5.0)
        self.declare_parameter('dna_db_path', '/tauv-mono/ros_ws/src/tauv_drivers/tauv_dronecan/dronecan_dna.db')
        self.declare_parameter('BIGARM',True)
        self.declare_paramter('ESC_MIN_VOLTAGE',13.0)
        interface = self.get_parameter('interface').value
        node_id = self.get_parameter('node_id').value
        bitrate = self.get_parameter('bitrate').value
        min_voltage = self.get_parameter('ESC_MIN_VOLTAGE').value
        self.esc_count = self.get_parameter('esc_count').value
        self.command_rate_hz = self.get_parameter('command_rate_hz').value
        discovery_time = self.get_parameter('discovery_time_sec').value
        dna_db_path = self.get_parameter('dna_db_path').value
        
        self.throttles = [0.0] * self.esc_count
        self.BIGARM=self.get_parameter('BIGARM').value
        self.armed = False
        self.discovered_escs = []
        self.telemetry = {}

        # Build remap and sign arrays from mapping.py
        # dronecan_to_global[dronecan_index] = global_index
        # dronecan_signs[dronecan_index] = sign for that thruster
        self.dronecan_to_global = [0] * self.esc_count
        self.dronecan_signs = [1] * self.esc_count
        for name, info in mapping.items():
            d_idx = info["dronecan_index"]
            g_idx = info["global_index"]
            self.dronecan_to_global[d_idx] = g_idx
            self.dronecan_signs[d_idx] = -1 if info["is_reverse"] else 1

        self.telemetry_pub = self.create_publisher(
            EscTelemetry,
            'esc_telemetry',
            10
        )
        
        # Initialize DroneCAN node
        self.get_logger().info(f'Initializing DroneCAN on {interface}...')
        node_info = dronecan.uavcan.protocol.GetNodeInfo.Response()
        node_info.name = 'CANDriverNode'
        node_info.software_version.major = 1

        
        
        self.dronecan_node = dronecan.make_node(
            interface,
            node_id=node_id,
            bitrate=bitrate,
            node_info=node_info
        )
        self.dronecan_node.mode = dronecan.uavcan.protocol.NodeStatus().MODE_OPERATIONAL
        
        # Initialize DNA server
        self.get_logger().info('Starting DNA server...')
        self.node_monitor = dronecan.app.node_monitor.NodeMonitor(self.dronecan_node)
        self.allocator = dronecan.app.dynamic_node_id.CentralizedServer(
            self.dronecan_node,
            self.node_monitor,
            database_storage=dna_db_path
        )
        
        # Add handlers
        self.dronecan_node.add_handler(
            dronecan.uavcan.equipment.esc.Status,
            self._on_esc_status
        )
        self.dronecan_node.add_handler(
            dronecan.uavcan.protocol.NodeStatus,
            self._on_node_status
        )
        
        self.get_logger().info("Discovering ESC nodes...")
        start_time = time.time()
        while ((time.time() - start_time) < discovery_time) or (len(self.discovered_escs) < self.esc_count):
            try:
                self.dronecan_node.spin(timeout=0.01)
            except dronecan.transport.TransferError:
                pass  
        
        self.get_logger().info(f'Found {len(self.discovered_escs)} ESCs: {self.discovered_escs}')
        self.arm()
        
        # Close DNA server after discovery
        if self.allocator:
            self.allocator.close()
            self.get_logger().info('DNA server closed')
        
        self.thruster_sub = self.create_subscription(
            ThrusterSetpoint,  # Replace with actual message type
            'thruster_rpms',
            self._thruster_callback,
            10
        )
        self.watchdog_sub= self.create_subscription(
            String,
            'watchdog/system_state',
            self._watchdog_callback,
            10
        )

        
        self.command_timer = self.create_timer(1.0 / self.command_rate_hz, self._send_commands)
        self.dronecan_timer = self.create_timer(0.001, self._spin_dronecan)
        
        self.get_logger().info('CANDriver node initialized')
    
    def _on_esc_status(self, event):
        node_id = event.transfer.source_node_id
        msg = event.message
        
        self.telemetry[node_id] = {
            'voltage': msg.voltage,
            'current': msg.current,
            'temperature': msg.temperature - 273.15 if msg.temperature > 0 else 0,
            'rpm': msg.rpm,
            'power_rating_pct': msg.power_rating_pct,
            'error_count': msg.error_count,
        }
        # self.get_logger().info(f'ESC {node_id} Telemetry: {self.telemetry[node_id]}')
        # Publish telemetry message
        telemetry_msg = EscTelemetry()
        telemetry_msg.id = node_id
        telemetry_msg.rpm = msg.rpm
        telemetry_msg.voltage = msg.voltage
        telemetry_msg.current = msg.current
        telemetry_msg.temperature = msg.temperature - 273.15 if msg.temperature > 0 else 0
        telemetry_msg.fault_code = msg.error_count

        self.telemetry_pub.publish(telemetry_msg)

    def avg_esc_voltage(self):
        if not self.telemetry:
            return 16.0  # Default voltage if no telemetry available
        return sum(t['voltage'] for t in self.telemetry.values()) / len(self.telemetry)

    def _on_node_status(self, event):
        node_id = event.transfer.source_node_id
        # print(f'Node {node_id} status update received')
        
        if node_id not in self.discovered_escs and node_id != self.dronecan_node.node_id:
            # Request node info to identify if it's an ESC
            def callback(event):
                
                if event and event.response:
                    name = bytes(event.response.name).decode().rstrip('\x00')

                    print(f'Node {node_id} responded with name: {name})')
                    # self.get_logger().info(f'Found ESC {node_id}: {name}')

                    if 'nanodrive' in name.lower(): #see how many nanodrives we have
                        self.discovered_escs.append(node_id)
                        self.discovered_escs.sort()
            
            self.dronecan_node.request(
                dronecan.uavcan.protocol.GetNodeInfo.Request(),
                node_id,
                callback
            )
    
    def _remap_thrusts(self, autonomy_thrusts):
        #make autonomy thrusts a normal array
        if isinstance(autonomy_thrusts, Float32MultiArray):
            autonomy_thrusts = list(autonomy_thrusts.data)
        elif isinstance(autonomy_thrusts, np.ndarray):
            autonomy_thrusts = autonomy_thrusts.tolist()   
            
        voltage = self.avg_esc_voltage()
        if voltage < self.get_parameter('ESC_MIN_VOLTAGE').value:
            self.get_logger().warn(f"ESC voltage {voltage:.2f}V below minimum threshold! Setting thrusts to zero. Disarming ESCs.")
            self.BIGARM = False
            return [0.0] * self.esc_count
        gain_thrusts = [
            rpm_to_gain(f, voltage) for f in autonomy_thrusts
        ]
        out = [
            gain_thrusts[self.dronecan_to_global[d_idx]] * self.dronecan_signs[d_idx]
            for d_idx in range(self.esc_count)
        ]
        # Convert forces to gains using the latest voltage telemetry
        # self.get_logger().info(f"type of autonomy_thrusts: {type(autonomy_thrusts)}, data: {autonomy_thrusts}")
        # self.get_logger().info(f"Remapped thrusts: {autonomy_thrusts} -> Gains: {gain_thrusts} at V={voltage:.2f}")
        return out

    def _watchdog_callback(self, msg):
        if msg.data == "OK":
            self.get_logger().debug("Received OK from watchdog")
            # self.BIGARM = True
        elif msg.data == "RESET":
            self.get_logger().debug("Received RESET from some cool person")
            self.BIGARM = True
        
        else:
            self.get_logger().warn(f"Unexpected watchdog message: '{msg.data}'")
            self.BIGARM = False

    def _thruster_callback(self, msg):

        if(len(msg.thrust) != self.esc_count):
            self.get_logger().warn(f'Received thrust array of length {len(msg.thrust)}, expected {self.esc_count}')
            return
        if(not msg.armed or not self.BIGARM):
            self.throttles = self._remap_thrusts(msg.thrust)
            self.disarm()
            return

        self.armed = True
        self.throttles = self._remap_thrusts(msg.thrust)

    def _send_commands(self):

        arming_msg = dronecan.uavcan.equipment.safety.ArmingStatus(
            status=255 if (self.armed and self.BIGARM) else 0
        )
        self.dronecan_node.broadcast(arming_msg)
                
        raw_values = [int(t * 1) for t in self.throttles]
        cmd_msg = dronecan.uavcan.equipment.esc.RawCommand(cmd=raw_values)
        self.dronecan_node.broadcast(cmd_msg)
    
    def _spin_dronecan(self):
        try:
            self.dronecan_node.spin(timeout=0.0)
        except dronecan.transport.TransferError:
            pass  # Ignore transfer errors
    
    def arm(self):
        
        self.armed = True
        self.get_logger().info('ESCs ARMED')
    
    def disarm(self):
        self.armed = False
        self.throttles = [0.0] * self.esc_count
        self.get_logger().info('ESCs DISARMED')
    
    def destroy_node(self):
        
        self.get_logger().info('Shutting down node')
        
        self.disarm()
        self._send_commands()
        time.sleep(0.1)  # Give time for message to send
        
        # Close DroneCAN resources
        if self.allocator:
            try:
                self.allocator.close()
            except:
                pass
        
        if self.node_monitor:
            try:
                self.node_monitor.close()
            except:
                pass
        
        if self.dronecan_node:
            self.dronecan_node.close()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CANDriver()
    
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