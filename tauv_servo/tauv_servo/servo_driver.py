#!/usr/bin/env python3

"""
Task-driven ROS 2 driver for Hitec MDB961WP-CAN servos.

Subscribes to a simple std_msgs/String on `command_topic` (default /servo/task).
The string names a *task* defined in config/servo_tasks.yaml; the node looks it
up and drives the mapped servo(s) to the mapped position(s) over CAN using the
hitec_servo library, then reads each servo back to verify it reached its target
angle. A std_msgs/String status is published on `status_topic`
(default /servo/status): "running_<task>" when a task starts, "done_<task>" once
every action has been verified, and "error_<task>" (with the reason logged) on
any failure.

On startup the node scans the CAN bus and logs every servo it finds, along with
a one-line telemetry snapshot for each.

Task config format (see config/servo_tasks.yaml):
    tasks:
      <task_name>:
        - {servo_id: <int>, position_deg: <0-360>}          # absolute
        - {servo_id: <int>, position_relative: <-150..150>}  # offset from center
        - {servo_id: <int>, position_deg: 90, torque_limit: 50}  # + torque cap
"""

import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from tauv_servo.hitec_servo import HitecBus


# An action carries exactly one of these. They differ only in what the angle is
# measured from: absolute, from center (180), or from where the servo was sitting
# when this node started.
_POSITION_KEYS = ('position_deg', 'position_relative', 'position_startup')


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

        # Parameters
        self.declare_parameter('interface', 'can0')
        self.declare_parameter('bitrate', 1000000)
        self.declare_parameter('tasks_config', _default_tasks_config())
        self.declare_parameter('command_topic', 'servo/task')
        self.declare_parameter('status_topic', 'servo/status')
        self.declare_parameter('torque_limit', 0.0)   # 0 = leave servo default
        self.declare_parameter('startup_task', '')     # '' = none
        self.declare_parameter('scan_max_id', 20)      # highest servo ID to probe
        self.declare_parameter('position_limit_deg', 150.0)  # +-limit set on init; 0 = leave as-is
        self.declare_parameter('save_limits', False)   # persist limits to flash (survives power cycle)
        self.declare_parameter('angle_tolerance', 3.0)  # deg — verify window
        self.declare_parameter('verify_timeout', 2.0)   # s — max wait to reach target
        self.declare_parameter('verify_poll', 0.1)      # s — readback interval
        self.declare_parameter('hold_on_startup', True)  # engage each servo at its as-found angle

        interface = self.get_parameter('interface').value
        bitrate = self.get_parameter('bitrate').value
        tasks_config = self.get_parameter('tasks_config').value
        command_topic = self.get_parameter('command_topic').value
        status_topic = self.get_parameter('status_topic').value
        self.torque_limit = float(self.get_parameter('torque_limit').value)
        startup_task = self.get_parameter('startup_task').value
        self.scan_max_id = int(self.get_parameter('scan_max_id').value)
        self.position_limit_deg = float(self.get_parameter('position_limit_deg').value)
        self.save_limits = bool(self.get_parameter('save_limits').value)
        self.angle_tolerance = float(self.get_parameter('angle_tolerance').value)
        self.verify_timeout = float(self.get_parameter('verify_timeout').value)
        self.verify_poll = float(self.get_parameter('verify_poll').value)
        self.hold_on_startup = bool(self.get_parameter('hold_on_startup').value)

        # Load task map + per-servo config from YAML
        config = self._load_config(tasks_config)
        self.tasks = config['tasks']
        self.servo_cfg = config['servos']
        self.get_logger().info(
            f'Loaded {len(self.tasks)} tasks: {sorted(self.tasks.keys())}')
        if self.servo_cfg:
            self.get_logger().info(
                f'Per-servo config for: {sorted(self.servo_cfg.keys())}')

        # Track which servos have had their torque limit applied (once each)
        self._torque_applied = set()

        # servo_id -> (min_deg, max_deg), read on first use
        self._limits = {}

        # servo_id -> the angle the servo was sitting at when this node started.
        # This is the zero reference for position_startup actions. Captured once
        # during the init scan and never updated, so a task is always measured
        # from where the mechanism began, not from wherever the last task left it.
        self._startup_deg = {}

        # Status publisher (create before bus so we can report bring-up failures)
        self.status_pub = self.create_publisher(String, status_topic, 10)

        # CAN bus bring-up — keep the node alive even if the bus fails to open,
        # so a bad cable/interface doesn't crash the whole launch.
        self.bus = None
        try:
            self.bus = HitecBus(interface, bitrate=bitrate)
            self.get_logger().info(
                f'HitecBus up on {interface} @ {bitrate} bps')
        except Exception as e:
            self.get_logger().error(
                f'Failed to open CAN bus on {interface}: {e}. '
                f'Node is up but commands will be rejected.')

        # Scan the bus and report every servo + its telemetry
        self._print_bus_info()

        # Command subscriber
        self.command_sub = self.create_subscription(
            String, command_topic, self._on_task, 10)

        self.get_logger().info(
            f'servo_driver ready — listening on "{command_topic}", '
            f'status on "{status_topic}"')

        # Optional startup task (e.g. center everything on boot)
        if startup_task:
            self.get_logger().info(f'Running startup task: {startup_task}')
            self._run_task(startup_task)

    # --- Bus info -------------------------------------------------------------

    def _print_bus_info(self):
        """Scan the CAN bus and log how many servos are present + telemetry."""
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
        for sid in found:
            servo = self.bus.servo(sid)

            # Capture the as-found angle FIRST, before we touch anything. This is
            # the zero for position_startup actions. The servo holds its physical
            # position across a reboot (absolute encoder, no homing), so this is
            # wherever the mechanism actually is right now.
            startup = servo.position()
            if startup is None:
                self.get_logger().warn(
                    f'  Servo {sid}: could not read startup position — '
                    f'position_startup actions for this servo will be rejected')
            else:
                self._startup_deg[sid] = startup
                self.get_logger().info(
                    f'  Servo {sid}: startup zero = {startup:.2f} deg')

            # Order matters. Cap torque first, so the motor can never engage at
            # full power. Then set the limits, so the guard rail is in place
            # before any move is possible. Only then engage the hold.
            self._apply_startup_torque(servo)
            self._apply_position_limits(servo)
            if self.hold_on_startup:
                self._hold_at_startup(servo)

            info = servo.info()
            if info:
                self.get_logger().info(f'  {info}')

            tele = servo.telemetry()
            if tele is not None:
                self.get_logger().info(f'  {tele}')
            else:
                self.get_logger().warn(f'  Servo {sid}: telemetry read failed')

    def _apply_position_limits(self, servo):
        """Set this servo's position limits during init.

        Per-servo config in the YAML wins over the global position_limit_deg
        parameter. A servo on a constrained mechanism gets limits centered on its
        STARTUP angle (limit_mode: startup), which makes its allowed travel a
        hardware-enforced guard rail: the servo physically clamps anything beyond
        it, so a bug in this node cannot drive the mechanism past the window.
        """
        cfg = self.servo_cfg.get(servo.id, {})
        limit_deg = float(cfg.get('limit_deg', self.position_limit_deg))
        mode = str(cfg.get('limit_mode', 'center')).strip().lower()

        if limit_deg <= 0:
            self._limits.pop(servo.id, None)  # fall back to reading the servo's own
            self.get_logger().info(
                f'  Servo {servo.id}: leaving existing position limits untouched')
            return

        if mode == 'startup':
            zero = self._startup_deg.get(servo.id)
            if zero is None:
                self.get_logger().error(
                    f'  Servo {servo.id}: limit_mode "startup" needs the startup '
                    f'angle, which could not be read — leaving limits untouched. '
                    f'This servo is NOT guard-railed.')
                self._limits.pop(servo.id, None)
                return
            center = zero
        elif mode == 'center':
            center = 180.0
        else:
            self.get_logger().error(
                f'  Servo {servo.id}: unknown limit_mode "{mode}" (expected '
                f'"center" or "startup") — leaving limits untouched.')
            self._limits.pop(servo.id, None)
            return

        lo = max(0.0, center - limit_deg)
        hi = min(360.0, center + limit_deg)

        if not servo.set_limits(lo, hi):
            self.get_logger().warn(
                f'  Servo {servo.id}: FAILED to set limits {lo:.1f}-{hi:.1f} deg')
            self._limits.pop(servo.id, None)
            return

        self._limits[servo.id] = (lo, hi)
        self.get_logger().info(
            f'  Servo {servo.id}: limits set to {lo:.1f}-{hi:.1f} deg '
            f'(+-{limit_deg:.0f} around {mode} {center:.1f})')

        if self.save_limits:
            servo.save()  # persists to flash (blocks ~1s); survives power cycle
            self.get_logger().info(f'  Servo {servo.id}: limits saved to flash')

    def _apply_startup_torque(self, servo):
        """Apply this servo's torque cap BEFORE the motor is ever engaged."""
        cfg = self.servo_cfg.get(servo.id, {})
        limit = float(cfg.get('torque_limit', self.torque_limit))
        if limit <= 0:
            return
        if servo.set_torque_limit(limit):
            self._torque_applied.add((servo.id, limit))
            self.get_logger().info(f'  Servo {servo.id}: torque capped at {limit:.0f}%')
        else:
            self.get_logger().warn(
                f'  Servo {servo.id}: FAILED to set torque cap {limit:.0f}%')

    def _hold_at_startup(self, servo):
        """Command the servo to hold the angle it was already sitting at.

        The servo boots with the motor idle (torque 0%) and only starts driving
        once it receives a position command, so a loaded mechanism can sag or
        back-drive until something engages it. This is a zero-distance move: it
        engages holding torque without moving the output.
        """
        zero = self._startup_deg.get(servo.id)
        if zero is None:
            return
        if servo.move(zero):
            self.get_logger().info(
                f'  Servo {servo.id}: holding at startup angle {zero:.2f} deg')
        else:
            self.get_logger().warn(
                f'  Servo {servo.id}: failed to engage hold at {zero:.2f} deg')

    # --- Config ---------------------------------------------------------------

    def _load_config(self, path: str) -> dict:
        """Load the YAML. Returns {'tasks': {...}, 'servos': {id: {...}}}."""
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

        raw_tasks = raw.get('tasks', {})
        if not isinstance(raw_tasks, dict):
            self.get_logger().error(
                f'{path}: top-level "tasks" must be a mapping.')
            raw_tasks = {}

        tasks = {}
        for name, actions in raw_tasks.items():
            valid = self._validate_actions(str(name), actions)
            if valid:
                tasks[str(name).strip().lower()] = valid

        return {'tasks': tasks, 'servos': self._validate_servo_cfg(path, raw)}

    def _validate_servo_cfg(self, path: str, raw: dict) -> dict:
        """Validate the optional per-servo section. Returns {servo_id: cfg}."""
        raw_servos = raw.get('servos', {}) or {}
        if not isinstance(raw_servos, dict):
            self.get_logger().error(
                f'{path}: "servos" must be a mapping — ignoring it. Servos will '
                f'fall back to the global position_limit_deg parameter.')
            return {}

        servos = {}
        for sid, cfg in raw_servos.items():
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

            mode = str(cfg.get('limit_mode', 'center')).strip().lower()
            if mode not in ('center', 'startup'):
                self.get_logger().warn(
                    f'{path}: servos[{sid}] limit_mode "{mode}" invalid '
                    f'(expected "center" or "startup") — skipping this servo, '
                    f'its limits will NOT be changed.')
                continue

            servos[sid] = cfg
        return servos

    def _validate_actions(self, name: str, actions) -> list:
        """Validate a task's action list; skip malformed actions with a warning."""
        if not isinstance(actions, list):
            self.get_logger().warn(
                f'Task "{name}": expected a list of actions, got '
                f'{type(actions).__name__} — skipping task.')
            return []

        valid = []
        for i, action in enumerate(actions):
            if not isinstance(action, dict):
                self.get_logger().warn(
                    f'Task "{name}" action {i}: not a mapping — skipping.')
                continue
            if 'servo_id' not in action:
                self.get_logger().warn(
                    f'Task "{name}" action {i}: missing servo_id — skipping.')
                continue

            keys = [k for k in _POSITION_KEYS if k in action]
            if len(keys) != 1:
                self.get_logger().warn(
                    f'Task "{name}" action {i}: need exactly one of '
                    f'{" / ".join(_POSITION_KEYS)} — got '
                    f'{keys if keys else "none"} — skipping.')
                continue

            valid.append(action)

        if not valid:
            self.get_logger().warn(f'Task "{name}": no valid actions.')
        return valid

    # --- Command handling -----------------------------------------------------

    def _on_task(self, msg: String):
        self._run_task(msg.data)

    def _run_task(self, raw_name: str):
        name = (raw_name or '').strip().lower()
        if not name:
            self.get_logger().warn('Received empty task name.')
            self._status('error_empty_task')
            return

        if self.bus is None:
            self.get_logger().error(f'Cannot run "{name}": CAN bus is down.')
            self._status(f'error_{name}')
            return

        actions = self.tasks.get(name)
        if actions is None:
            self.get_logger().warn(f'Unknown task: "{name}"')
            self._status(f'error_{name}')
            return

        self.get_logger().info(f'Running task "{name}" ({len(actions)} action(s))')
        self._status(f'running_{name}')

        ok = True
        for action in actions:
            if not self._run_action(name, action):
                ok = False

        if ok:
            self.get_logger().info(f'Task "{name}" done (all angles verified)')
            self._status(f'done_{name}')
        else:
            self.get_logger().warn(f'Task "{name}" failed.')
            self._status(f'error_{name}')

    def _run_action(self, task_name: str, action: dict) -> bool:
        """Execute a single validated servo action. Returns True on ack."""
        servo_id = action['servo_id']
        try:
            servo = self.bus.servo(servo_id)
        except Exception as e:
            self.get_logger().error(
                f'Task "{task_name}": bad servo_id {servo_id}: {e}')
            return False

        # Per-servo torque limit: from the action, else the global default.
        # Applied once per servo (until it changes) to avoid redundant writes.
        limit = action.get('torque_limit', self.torque_limit)
        if limit and (servo_id, limit) not in self._torque_applied:
            if servo.set_torque_limit(float(limit)):
                self._torque_applied.add((servo_id, limit))
                self.get_logger().info(
                    f'Servo {servo_id}: torque limit set to {limit}%')
            else:
                self.get_logger().warn(
                    f'Servo {servo_id}: failed to set torque limit {limit}%')

        # Every mode resolves to one absolute target, so the range check, the
        # move and the readback verification are all done the same way.
        if 'position_deg' in action:
            target_abs = float(action['position_deg'])
            desc = f'{target_abs:.1f} deg abs'
        elif 'position_relative' in action:
            offset = float(action['position_relative'])
            target_abs = 180.0 + offset
            desc = f'{offset:+.1f} deg from center ({target_abs:.1f} deg abs)'
        else:
            offset = float(action['position_startup'])
            zero = self._startup_deg.get(servo_id)
            if zero is None:
                self.get_logger().error(
                    f'Task "{task_name}": servo {servo_id} has no startup zero '
                    f'(it was not seen during the init scan) — cannot resolve '
                    f'position_startup: {offset:+.1f}')
                return False
            target_abs = zero + offset
            desc = (f'{offset:+.1f} deg from startup zero {zero:.1f} '
                    f'({target_abs:.1f} deg abs)')

        # The servo CLAMPS an out-of-limit target rather than refusing it: it
        # would drive to the limit and stop, acking with no error flag. That is
        # real motion to a position nobody asked for, so refuse to send it.
        if not self._within_limits(servo, target_abs, desc):
            return False

        # Always command the absolute target we just computed. Using the servo's
        # own move_relative() here would silently re-reference the offset to
        # center instead of the startup zero.
        if not servo.move(target_abs):
            self.get_logger().warn(
                f'Servo {servo_id}: move to {desc} not acknowledged (no response)')
            return False

        self.get_logger().info(f'Servo {servo_id} -> {desc}; verifying...')
        return self._verify_angle(servo, target_abs, desc)

    def _within_limits(self, servo, target_abs: float, desc: str) -> bool:
        """True if target_abs is inside the servo's position limits.

        Limits are read once per servo and cached; they only change if something
        rewrites them, which only this node does (at init).
        """
        if servo.id not in self._limits:
            limits = servo.get_limits()
            if limits is None:
                self.get_logger().warn(
                    f'Servo {servo.id}: could not read position limits; '
                    f'skipping range check')
                return True
            self._limits[servo.id] = limits
        lo, hi = self._limits[servo.id]

        # Tolerance-width slack so a target set exactly at the limit (e.g. a
        # setpoint recorded there) isn't rejected by rounding.
        if lo - self.angle_tolerance <= target_abs <= hi + self.angle_tolerance:
            return True

        self.get_logger().error(
            f'Servo {servo.id}: target {desc} is outside its position limits '
            f'({lo:.1f}-{hi:.1f} deg). Refusing — the servo would clamp and '
            f'drive to the limit instead.')
        return False

    def _verify_angle(self, servo, target_abs: float, desc: str) -> bool:
        """Poll the servo position until it reaches target_abs within tolerance."""
        deadline = time.monotonic() + self.verify_timeout
        last = None
        while time.monotonic() < deadline:
            pos = servo.position()
            if pos is not None:
                last = pos
                if abs(pos - target_abs) <= self.angle_tolerance:
                    self.get_logger().info(
                        f'Servo {servo.id}: reached {pos:.1f} deg '
                        f'(target {target_abs:.1f}, tol {self.angle_tolerance})')
                    return True
            time.sleep(self.verify_poll)

        last_str = f'{last:.1f} deg' if last is not None else 'read failed'
        self.get_logger().warn(
            f'Servo {servo.id}: did NOT reach {desc} within '
            f'{self.verify_timeout}s (last={last_str}, target={target_abs:.1f}, '
            f'tol={self.angle_tolerance})')
        return False

    # --- Helpers --------------------------------------------------------------

    def _status(self, text: str):
        self.status_pub.publish(String(data=text))

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
