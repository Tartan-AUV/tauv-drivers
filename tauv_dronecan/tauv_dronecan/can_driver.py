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
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
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
        self.declare_parameter('BIGARM', True)
        self.declare_parameter('telemetry_rate_hz', 6.0)
        self.declare_parameter('discovery_time_sec', 5.0)
        self.declare_parameter('dna_db_path', '/tauv-mono/ros_ws/src/tauv_drivers/tauv_dronecan/dronecan_dna.db')
        # Undervoltage protection, on the average pack voltage reported by the ESCs.
        self.declare_parameter('startup_voltage_min_v', 14.0)
        self.declare_parameter('undervoltage_cutoff_v', 12.3)
        self.declare_parameter('undervoltage_hold_sec', 5.0)
        self.declare_parameter('watchdog_topic', 'watchdog/system_state')
        self.declare_parameter('watchdog_timeout_s', 1.5)
        self.declare_parameter('watchdog_require_rearm_edge', False)

        interface = self.get_parameter('interface').value
        node_id = self.get_parameter('node_id').value
        bitrate = self.get_parameter('bitrate').value
        self.esc_count = self.get_parameter('esc_count').value
        self.command_rate_hz = self.get_parameter('command_rate_hz').value
        telemetry_rate_hz = self.get_parameter('telemetry_rate_hz').value
        self.startup_voltage_min_v = self.get_parameter('startup_voltage_min_v').value
        self.undervoltage_cutoff_v = self.get_parameter('undervoltage_cutoff_v').value
        self.undervoltage_hold_sec = self.get_parameter('undervoltage_hold_sec').value
        # Monotonic timestamp of the first sample below the cutoff, or None if the
        # pack is currently above it.
        self._undervoltage_since = None
        self.watchdog_timeout_s = self.get_parameter('watchdog_timeout_s').value
        if not math.isfinite(self.watchdog_timeout_s) or self.watchdog_timeout_s <= 0:
            raise ValueError('watchdog_timeout_s must be finite and positive')
        self.watchdog_require_rearm_edge = self.get_parameter(
            'watchdog_require_rearm_edge').value
        self._watchdog_ok = False
        self._last_watchdog_time = None
        # A healthy watchdog grants permission, never an arm request.
        self._rearm_ready = False
        # Minimum seconds between esc_telemetry publishes per ESC (0 => unthrottled).
        self.telemetry_period_s = 1.0 / telemetry_rate_hz if telemetry_rate_hz > 0.0 else 0.0
        self._last_telem_pub_time = {}
        discovery_time = self.get_parameter('discovery_time_sec').value
        dna_db_path = self.get_parameter('dna_db_path').value
        
        self.throttles = [0.0] * self.esc_count
        self.BIGARM= self.get_parameter('BIGARM').value
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
        start_time = time.monotonic()
        while ((time.monotonic() - start_time) < discovery_time) or (len(self.discovered_escs) < self.esc_count):
            try:
                self.dronecan_node.spin(timeout=0.01)
            except dronecan.transport.TransferError:
                pass  
        
        self.get_logger().info(f'Found {len(self.discovered_escs)} ESCs: {self.discovered_escs}')

        # Preserve discovery/DNA lifetime; battery health alone cannot arm.
        # The original discovery loop waits for every configured ESC.
        self.disarm()
        if not self._startup_voltage_ok():
            # Latched refusal: nothing sets BIGARM back to True, so the node runs
            # (telemetry keeps publishing) but never commands the thrusters.
            self.BIGARM = False
            self.disarm()

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
            self.get_parameter('watchdog_topic').value,
            self._watchdog_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)
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

        # Rate-limit the ROS publish per ESC (ESCs broadcast status faster than we
        # want on the topic). Internal self.telemetry above stays at full rate.
        now = time.monotonic()
        last = self._last_telem_pub_time.get(node_id)
        if self.telemetry_period_s > 0.0 and last is not None and \
                (now - last) < self.telemetry_period_s:
            return
        self._last_telem_pub_time[node_id] = now

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

    def _startup_voltage_ok(self):
        """Whether the pack is healthy enough to arm at all.

        Checked once after discovery. Without telemetry we cannot see the pack at
        all, so we refuse rather than trust the 16.0 V default in avg_esc_voltage.
        """
        if not self.telemetry:
            self.get_logger().error(
                'NO ESC TELEMETRY - cannot read pack voltage, REFUSING TO ARM'
            )
            return False

        voltage = self.avg_esc_voltage()
        if voltage < self.startup_voltage_min_v:
            self.get_logger().error(
                f'PACK UNDERVOLTAGE AT STARTUP: {voltage:.2f} V is below the '
                f'{self.startup_voltage_min_v:.2f} V minimum'
            )
            self.get_logger().error(
                'REFUSING TO ARM - charge or swap the battery, then restart this node'
            )
            return False

        self.get_logger().info(f'Pack voltage OK at startup: {voltage:.2f} V')
        return True

    def _check_undervoltage(self):
        """Kill the thrusters if the pack holds below the cutoff for undervoltage_hold_sec.

        This clears BIGARM independently of the resettable watchdog gate.
        Watchdog recovery cannot clear this battery lockout; restart is required.
        """
        if not self.BIGARM:
            return  # already killed
        if not self.telemetry:
            return  # no telemetry yet; avg_esc_voltage would report its 16.0 V default

        voltage = self.avg_esc_voltage()
        now = time.monotonic()

        if voltage >= self.undervoltage_cutoff_v:
            if self._undervoltage_since is not None:
                self.get_logger().info(f'Pack voltage recovered to {voltage:.2f} V')
                self._undervoltage_since = None
            return

        if self._undervoltage_since is None:
            self._undervoltage_since = now
            self.get_logger().warn(
                f'Pack at {voltage:.2f} V, below the {self.undervoltage_cutoff_v:.2f} V '
                f'cutoff - killing thrusters if this holds {self.undervoltage_hold_sec:.1f} s'
            )
            return

        held = now - self._undervoltage_since
        if held >= self.undervoltage_hold_sec:
            self.get_logger().error(
                f'UNDERVOLTAGE: pack at {voltage:.2f} V for {held:.1f} s - KILLING THRUSTERS'
            )
            self.BIGARM = False
            self.disarm()

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

    def _watchdog_stop(self, reason):
        if self._watchdog_ok or self.armed:
            self.get_logger().warn(reason)
        self._watchdog_ok = False
        self._rearm_ready = False
        self.disarm()

    def _watchdog_is_fresh(self, now):
        return (self._watchdog_ok and self._last_watchdog_time is not None
                and now - self._last_watchdog_time < self.watchdog_timeout_s)

    def _watchdog_callback(self, msg):
        now = time.monotonic()
        # Detect gaps even if a recovered heartbeat arrives before the timer.
        if not self._watchdog_is_fresh(now):
            self._watchdog_stop('Watchdog unavailable or expired')
        self._last_watchdog_time = now
        # Canonical protocol is continue/kill; accept the previous OK/ERROR
        # producer during migration. Every other value fails closed.
        if msg.data in ("continue", "OK"):
            self._watchdog_ok = True
        else:
            self._watchdog_stop(f'Watchdog reported {msg.data!r}')
        # Send zero commands immediately on kill; continue never arms.
        if not self._watchdog_ok:
            self._send_commands()

    def _thruster_callback(self, msg):
        if not self._watchdog_is_fresh(time.monotonic()):
            self._watchdog_stop('Watchdog unavailable or expired')
            return
        if not self.BIGARM:
            self._rearm_ready = False
            self.disarm()
            return
        if len(msg.thrust) != self.esc_count:
            self.get_logger().warn(
                f'Received thrust array of length {len(msg.thrust)}, expected {self.esc_count}')
            return
        if not msg.armed:
            self.disarm()
            self._rearm_ready = True
            return
        if self.watchdog_require_rearm_edge and not self._rearm_ready:
            self.disarm()
            return
        throttles = self._remap_thrusts(msg.thrust)
        self.arm()
        self.throttles = throttles if self.armed else [0.0] * self.esc_count

    def _send_commands(self):

        # Runs before the broadcast below, so the tick that trips the cutoff already
        # sends ArmingStatus=0 and zeroed throttles.
        self._check_undervoltage()

        if not self._watchdog_is_fresh(time.monotonic()):
            self._watchdog_stop('Watchdog unavailable or expired')
        armed = self.armed and self.BIGARM and self._watchdog_ok

        arming_msg = dronecan.uavcan.equipment.safety.ArmingStatus(
            status=255 if armed else 0
        )
        self.dronecan_node.broadcast(arming_msg)

        # All safety gates are authoritative on the wire: never put a nonzero
        # command on the bus, no matter what left self.throttles set.
        throttles = self.throttles if armed else [0.0] * self.esc_count
        raw_values = [int(t * 1) for t in throttles]
        cmd_msg = dronecan.uavcan.equipment.esc.RawCommand(cmd=raw_values)
        self.dronecan_node.broadcast(cmd_msg)
    
    def _spin_dronecan(self):
        try:
            self.dronecan_node.spin(timeout=0.0)
        except dronecan.transport.TransferError:
            pass  # Ignore transfer errors
    
    def arm(self):
        
        if (self.BIGARM
                and (not self.watchdog_require_rearm_edge or self._rearm_ready)
                and self._watchdog_is_fresh(time.monotonic())):
            if not self.armed:
                self.get_logger().info('ESCs ARMED')
            self.armed = True
        else:
            self.disarm()
    
    def disarm(self):
        self.armed = False
        self.throttles = [0.0] * self.esc_count
        self.get_logger().info('ESCs DISARMED', throttle_duration_sec=2.0)
    
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
