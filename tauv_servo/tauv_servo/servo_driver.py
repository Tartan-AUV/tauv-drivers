#!/usr/bin/env python3

"""
ROS 2 driver for Hitec MDB961WP-CAN servos.

Publish a task name (std_msgs/String) on `command_topic` (default /servo/task).
The task is looked up in config/servo_tasks.yaml, which maps it to one or more
{servo_id, angle} actions. `angle` is an ABSOLUTE angle in degrees, 0-360,
where 180 is the servo's center. The node commands each move over CAN, then
reads the servo back to verify it actually reached the angle.

Progress is published as tauv_msgs/Status on `status_topic`
(default /mission/status):

    id      mechanism name from the servo's `id:` in the YAML
            (e.g. "torpedo" = servo 1, "dropper" = servo 2)
    status  2 = running (task started)
            1 = success (every angle read back and verified)
            0 = failed  (unknown task, bus down, no ack, out of limits,
                         or angle not reached)

On startup the node scans the CAN bus, applies each servo's torque cap and
position limits from the YAML, and engages holding torque at each servo's
as-found angle (the servo boots with its motor idle, so a loaded mechanism
would otherwise sag until the first command).
"""

import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)
from std_msgs.msg import String
from tauv_msgs.msg import ServoTelemetry, Status

from tauv_servo.hitec_servo import HitecBus

# tauv_msgs/Status.status values.
STATUS_FAILED = 0
STATUS_SUCCESS = 1
STATUS_RUNNING = 2


def _default_tasks_config() -> str:
    """Installed config/servo_tasks.yaml path, or '' if it can't be resolved."""
    try:
        return os.path.join(
            get_package_share_directory('tauv_servo'),
            'config', 'servo_tasks.yaml')
    except Exception:
        return ''


class ServoDriver(Node):
    def __init__(self):
        super().__init__('servo_driver')

        self.declare_parameter('interface', 'can0')
        self.declare_parameter('bitrate', 1000000)
        self.declare_parameter('tasks_config', _default_tasks_config())
        self.declare_parameter('command_topic', 'servo/task')
        self.declare_parameter('status_topic', '/mission/status')
        self.declare_parameter('torque_limit', 0.0)   # % cap; 0 = leave servo default
        self.declare_parameter('startup_task', '')    # '' = none
        self.declare_parameter('scan_max_id', 3)     # highest servo ID to probe
        self.declare_parameter('position_limit_deg', 150.0)  # fallback +-window around 180
        self.declare_parameter('angle_tolerance', 3.0)  # deg — verify window
        self.declare_parameter('verify_timeout', 2.0)   # s — max wait to reach target
        self.declare_parameter('verify_poll', 0.1)      # s — readback interval
        self.declare_parameter('telem_rate_hz', 1.0)    # 0 = no periodic telemetry
        self.declare_parameter('telem_topic_prefix', '/servo/telem')

        interface = self.get_parameter('interface').value
        bitrate = self.get_parameter('bitrate').value
        tasks_config = self.get_parameter('tasks_config').value
        command_topic = self.get_parameter('command_topic').value
        status_topic = self.get_parameter('status_topic').value
        startup_task = self.get_parameter('startup_task').value
        self.torque_limit = float(self.get_parameter('torque_limit').value)
        self.scan_max_id = int(self.get_parameter('scan_max_id').value)
        self.position_limit_deg = float(self.get_parameter('position_limit_deg').value)
        self.angle_tolerance = float(self.get_parameter('angle_tolerance').value)
        self.verify_timeout = float(self.get_parameter('verify_timeout').value)
        self.verify_poll = float(self.get_parameter('verify_poll').value)
        self.telem_rate_hz = float(self.get_parameter('telem_rate_hz').value)
        self.telem_prefix = str(self.get_parameter('telem_topic_prefix').value).rstrip('/')

        # Load task map + per-servo config from YAML
        config = self._load_config(tasks_config)
        self.tasks = config['tasks']
        self.servo_cfg = config['servos']
        self.get_logger().info(
            f'Loaded {len(self.tasks)} tasks: {sorted(self.tasks.keys())}')

        # servo_id -> (min_deg, max_deg) as applied at init (or read on demand)
        self._limits = {}
        # Servo IDs found by the init scan
        self._servo_ids = []
        # servo_id -> telemetry publisher, created on demand
        self._telem_pubs = {}

        # Status publisher (create before bus so we can report bring-up failures).
        # Latched (RELIABLE + TRANSIENT_LOCAL): a status published before the
        # mission planner finishes discovering this publisher — or while it is
        # (re)starting — is delivered as soon as it matches instead of being
        # lost. With only two messages per task, a lost one means the planner
        # waits forever.
        status_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(Status, status_topic, status_qos)

        # CAN bus bring-up — keep the node alive even if the bus fails to open,
        # so a bad cable/interface doesn't crash the whole launch.
        self.bus = None
        try:
            self.bus = HitecBus(interface, bitrate=bitrate)
            self.get_logger().info(f'HitecBus up on {interface} @ {bitrate} bps')
        except Exception as e:
            self.get_logger().error(
                f'Failed to open CAN bus on {interface}: {e}. '
                f'Node is up but commands will be rejected.')

        # Scan the bus, then configure and report every servo found
        self._init_servos()

        # Periodic telemetry poll
        if self.telem_rate_hz > 0 and self.bus is not None:
            self.create_timer(1.0 / self.telem_rate_hz, self._poll_telemetry)
            self.get_logger().info(
                f'Telemetry at {self.telem_rate_hz:.1f} Hz on {self.telem_prefix}/<id>')

        # Command subscriber
        self.command_sub = self.create_subscription(
            String, command_topic, self._on_task, 10)

        self.get_logger().info(
            f'servo_driver ready — listening on "{command_topic}", '
            f'status on "{status_topic}"')

        if startup_task:
            self.get_logger().info(f'Running startup task: {startup_task}')
            self._run_task(startup_task)

    # --- Startup --------------------------------------------------------------

    def _init_servos(self):
        """Scan the bus; apply torque cap + position limits to each servo found."""
        if self.bus is None:
            self.get_logger().warn('CAN bus down — skipping servo scan.')
            return

        self.get_logger().info(f'Scanning CAN bus (IDs 1-{self.scan_max_id})...')
        try:
            found = self.bus.scan(self.scan_max_id)
        except Exception as e:
            self.get_logger().error(f'Bus scan failed: {e}')
            return

        if not found:
            self.get_logger().warn(
                'No servos found on the bus. Check wiring/power/sample-point.')
            return

        self.get_logger().info(f'Found {len(found)} servo(s): {found}')
        self._servo_ids = list(found)
        for sid in found:
            servo = self.bus.servo(sid)
            cfg = self.servo_cfg.get(sid, {})

            # Torque cap FIRST, so the motor can never engage at full power.
            torque = float(cfg.get('torque_limit', self.torque_limit))
            if torque > 0:
                if servo.set_torque_limit(torque):
                    self.get_logger().info(
                        f'  Servo {sid}: torque capped at {torque:.0f}%')
                else:
                    self.get_logger().warn(
                        f'  Servo {sid}: FAILED to set torque cap {torque:.0f}%')

            # Position limits: enforced by the servo itself, so a bug in this
            # node cannot drive a mechanism past its window.
            lo, hi = self._servo_window(sid)
            if servo.set_limits(lo, hi):
                self._limits[sid] = (lo, hi)
                self.get_logger().info(
                    f'  Servo {sid}: limits set to {lo:.1f}-{hi:.1f} deg')
            else:
                self.get_logger().warn(
                    f'  Servo {sid}: FAILED to set limits {lo:.1f}-{hi:.1f} deg')

            # Engage holding torque at the as-found angle (zero-distance move).
            # Skipped if the servo is resting outside its window — commanding it
            # would be clamped and actually move the mechanism.
            pos = servo.position()
            if pos is None:
                self.get_logger().warn(f'  Servo {sid}: position read failed')
            elif not (lo <= pos <= hi):
                self.get_logger().error(
                    f'  Servo {sid}: resting at {pos:.1f} deg, OUTSIDE its '
                    f'{lo:.1f}-{hi:.1f} window — check min/max_angle in the '
                    f'YAML. Not engaging hold.')
            elif servo.move(pos):
                self.get_logger().info(
                    f'  Servo {sid}: holding at {pos:.2f} deg')

            info = servo.info()
            if info:
                self.get_logger().info(f'  {info}')

            tele = servo.telemetry()
            if tele is not None:
                self.get_logger().info(f'  {tele}')
                self._publish_telem(sid, tele)
            else:
                self.get_logger().warn(f'  Servo {sid}: telemetry read failed')

    def _servo_window(self, servo_id: int) -> tuple:
        """This servo's allowed (min_deg, max_deg) window.

        Per-servo min_angle/max_angle from the YAML win; otherwise the global
        position_limit_deg parameter, as +-window around center (180).
        """
        cfg = self.servo_cfg.get(servo_id, {})
        if 'min_angle' in cfg and 'max_angle' in cfg:
            return (float(cfg['min_angle']), float(cfg['max_angle']))
        lo = max(0.0, 180.0 - self.position_limit_deg)
        hi = min(360.0, 180.0 + self.position_limit_deg)
        return (lo, hi)

    # --- Config ---------------------------------------------------------------

    def _load_config(self, path: str) -> dict:
        """Load the YAML. Returns {'tasks': {name: [(servo_id, angle)]},
        'servos': {servo_id: cfg}}."""
        empty = {'tasks': {}, 'servos': {}}

        if not path:
            self.get_logger().error(
                'No tasks_config parameter set — no tasks available.')
            return empty
        if not os.path.isfile(path):
            self.get_logger().error(
                f'tasks_config not found: {path} — no tasks available.')
            return empty

        try:
            with open(path, 'r') as f:
                raw = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f'Failed to parse {path}: {e}')
            return empty

        servos = {}
        for sid, cfg in (raw.get('servos') or {}).items():
            try:
                sid = int(sid)
            except (TypeError, ValueError):
                self.get_logger().warn(
                    f'{path}: servos key "{sid}" is not an int — skipping.')
                continue
            if not isinstance(cfg, dict):
                self.get_logger().warn(
                    f'{path}: servos[{sid}] must be a mapping — skipping.')
                continue
            servos[sid] = cfg

        tasks = {}
        for name, actions in (raw.get('tasks') or {}).items():
            valid = self._validate_actions(str(name), actions)
            if valid:
                tasks[str(name).strip().lower()] = valid

        return {'tasks': tasks, 'servos': servos}

    def _validate_actions(self, name: str, actions) -> list:
        """Validate one task. Returns a list of (servo_id, angle) tuples."""
        if not isinstance(actions, list):
            self.get_logger().warn(
                f'Task "{name}": expected a list of actions — skipping task.')
            return []

        valid = []
        for i, action in enumerate(actions):
            if not isinstance(action, dict):
                self.get_logger().warn(
                    f'Task "{name}" action {i}: not a mapping — skipping.')
                continue
            try:
                sid = int(action['servo_id'])
                angle = float(action['angle'])
            except (KeyError, TypeError, ValueError):
                self.get_logger().warn(
                    f'Task "{name}" action {i}: needs servo_id and angle '
                    f'(got {action}) — skipping.')
                continue
            if not 0.0 <= angle <= 360.0:
                self.get_logger().warn(
                    f'Task "{name}" action {i}: angle {angle} outside 0-360 '
                    f'— skipping.')
                continue
            valid.append((sid, angle))

        if not valid:
            self.get_logger().warn(f'Task "{name}": no valid actions.')
        return valid

    # --- Command handling -------------------------------------------------------

    def _on_task(self, msg: String):
        self._run_task(msg.data)

    def _run_task(self, raw_name: str):
        name = (raw_name or '').strip().lower()
        if not name:
            self.get_logger().warn('Received empty task name.')
            self._status([], STATUS_FAILED)
            return

        actions = self.tasks.get(name)
        if actions is None:
            self.get_logger().warn(f'Unknown task: "{name}"')
            self._status([], STATUS_FAILED)
            return

        # Resolve the mechanism id(s) up front so a failure is reported against
        # the right mechanism.
        ids = self._task_ids(actions)

        if self.bus is None:
            self.get_logger().error(f'Cannot run "{name}": CAN bus is down.')
            self._status(ids, STATUS_FAILED)
            return
#dont publish 2 if already at angle

        if all([abs(self.bus.servo(sid).position() - angle) <= self.angle_tolerance for sid, angle in actions]):
            self.get_logger().info(f'Task "{name}": all servos already at target angles.')
            self._status(ids, STATUS_SUCCESS)
            return
        self.get_logger().info(
            f'Running task "{name}" ({len(actions)} action(s)) -> {ids or "no id"}')
        self._status(ids, STATUS_RUNNING)

        ok = all([self._move_and_verify(sid, angle) for sid, angle in actions])
        if ok:
            self.get_logger().info(f'Task "{name}" done (all angles verified)')
            self._status(ids, STATUS_SUCCESS)
        else:
            self.get_logger().warn(f'Task "{name}" failed.')
            self._status(ids, STATUS_FAILED)

    def _move_and_verify(self, servo_id: int, angle: float) -> bool:
        """Move one servo to an absolute angle and verify it got there."""
        servo = self.bus.servo(servo_id)

        # The servo CLAMPS an out-of-limit target rather than refusing it: it
        # drives to the limit, stops, and acks with no error flag. That is real
        # motion to a position nobody asked for, so refuse to send it.
        if not self._within_limits(servo, angle):
            return False

        if not servo.move(angle):
            self.get_logger().warn(
                f'Servo {servo_id}: move to {angle:.1f} deg not acknowledged '
                f'(no response)')
            return False

        self.get_logger().info(f'Servo {servo_id} -> {angle:.1f} deg; verifying...')
        return self._verify_angle(servo, angle)

    def _within_limits(self, servo, angle: float) -> bool:
        """True if angle is inside the servo's position limits."""
        if servo.id not in self._limits:
            limits = servo.get_limits()
            if limits is None:
                self.get_logger().warn(
                    f'Servo {servo.id}: could not read position limits; '
                    f'skipping range check')
                return True
            self._limits[servo.id] = limits
        lo, hi = self._limits[servo.id]

        # Tolerance-width slack so a target recorded exactly at a limit isn't
        # rejected by rounding.
        if lo - self.angle_tolerance <= angle <= hi + self.angle_tolerance:
            return True

        self.get_logger().error(
            f'Servo {servo.id}: target {angle:.1f} deg is outside its limits '
            f'({lo:.1f}-{hi:.1f} deg). Refusing — the servo would clamp and '
            f'drive to the limit instead.')
        return False

    def _verify_angle(self, servo, angle: float) -> bool:
        """Poll the servo position until it reaches the angle within tolerance."""
        deadline = time.monotonic() + self.verify_timeout
        last = None
        while time.monotonic() < deadline:
            pos = servo.position()
            if pos is not None:
                last = pos
                if abs(pos - angle) <= self.angle_tolerance:
                    self.get_logger().info(
                        f'Servo {servo.id}: reached {pos:.1f} deg '
                        f'(target {angle:.1f}, tol {self.angle_tolerance})')
                    return True
            time.sleep(self.verify_poll)

        last_str = f'{last:.1f} deg' if last is not None else 'read failed'
        self.get_logger().warn(
            f'Servo {servo.id}: did NOT reach {angle:.1f} deg within '
            f'{self.verify_timeout}s (last={last_str}, tol={self.angle_tolerance})')
        return False

    # --- Status -----------------------------------------------------------------

    def _task_ids(self, actions) -> list:
        """The mechanism ids a task drives, e.g. ['torpedo'].

        From the `id:` field of each servo in the YAML `servos:` section.
        Order-preserving and de-duped.
        """
        ids = []
        for sid, _ in actions:
            mech = self.servo_cfg.get(sid, {}).get('id')
            if mech and mech not in ids:
                ids.append(str(mech))
        return ids

    def _status(self, ids, code: int):
        """Publish one tauv_msgs/Status per mechanism id."""
        if not ids:
            # No mechanism could be resolved (unknown task, or a servo with no
            # id: in the config). Still report, with an empty id, so a
            # subscriber sees the failure rather than silence.
            ids = ['']
        for mech in ids:
            self.status_pub.publish(Status(id=mech, status=code))
            self.get_logger().info(f'Status published: id="{mech}" status={code}')

    # --- Telemetry ----------------------------------------------------------------

    def _publish_telem(self, servo_id: int, tele) -> None:
        """Publish one telemetry reading (publisher created on first use).

        A None reading is dropped rather than published as zeros — a servo that
        stops answering should show up as a gap in the topic, not as a
        plausible-looking 0 V / 0 deg message.
        """
        if tele is None:
            return

        pub = self._telem_pubs.get(servo_id)
        if pub is None:
            topic = f'{self.telem_prefix}/servo{servo_id}'
            pub = self.create_publisher(ServoTelemetry, topic, 10)
            self._telem_pubs[servo_id] = pub
            self.get_logger().info(f'  Servo {servo_id}: telemetry on {topic}')

        msg = ServoTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'servo_{servo_id}'
        msg.servo_id = int(servo_id)
        msg.position_deg = float(tele.position_deg)
        msg.velocity = float(tele.velocity)
        msg.torque_percent = float(tele.torque_percent)
        msg.voltage = float(tele.voltage)
        msg.temperature_c = int(tele.temperature_c)
        msg.current_ma = int(tele.current_ma)
        msg.errors = [str(e) for e in tele.errors]
        pub.publish(msg)

    def _poll_telemetry(self):
        """Timer: read every servo and publish.

        Single-threaded executor, so this never overlaps a task callback — a
        long task simply delays a telemetry tick.
        """
        for sid in self._servo_ids:
            try:
                tele = self.bus.servo(sid).telemetry()
            except Exception as e:
                self.get_logger().warn(f'Servo {sid}: telemetry read raised {e}')
                continue
            self._publish_telem(sid, tele)

    def destroy_node(self):
        self.get_logger().info('Shutting down servo_driver')
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()

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
