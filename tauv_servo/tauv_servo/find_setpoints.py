#!/usr/bin/env python3
"""
find_setpoints.py — interactive helper for discovering servo positions for
config/servo_tasks.yaml.

It does NOT touch the YAML file. It lets you jog servos to angles, reads back
where they actually landed, and prints ready-to-paste YAML snippets for the
tasks you build up.

Run:
    python3 find_setpoints.py [interface]      # default interface: can0

Typical workflow:
    1. It scans the bus and shows every servo it finds.
    2. `use 1`            -> select servo 1
    3. `unlock`          -> open the full +-150 deg range (so moves aren't ignored)
    4. `a 250`           -> move to 250 deg absolute; it reads back the real angle
    5. `+ 10` / `- 5`    -> nudge from the current position until it looks right
    6. `add drop_marker` -> record servo 1's current angle under task "drop_marker"
    7. ...repeat for other servos/tasks...
    8. `yaml`            -> print the full tasks: block to paste into servo_tasks.yaml

Type `help` for the full command list.
"""

import sys
import time

# Works both as a package module (ros2 run / colcon) and when run directly
# from this directory (python3 find_setpoints.py ...).
try:
    from tauv_servo.hitec_servo import HitecBus
except ImportError:
    from hitec_servo import HitecBus

INTERFACE = "can0"

HELP = """
Commands (angles are in degrees; 180 = center):
  scan                 Re-scan the bus and list servos
  use <id>             Select which servo to control
  a <deg>              Move to ABSOLUTE angle (0-360)
  r <deg>              Move RELATIVE to center (-150..+150)
  + <deg>              Nudge current position up by <deg>
  - <deg>              Nudge current position down by <deg>
  p                    Print current position (abs + relative)
  t                    Print full telemetry
  unlock               Open full +-150 deg limits (30-330) on selected servo
  limits <min> <max>   Set custom absolute limits (deg)
  torque <pct>         Set torque limit (0-100 %) on selected servo
  save                 Persist current servo settings to flash (survives reboot)

  add <task> [id]      Record a servo's CURRENT angle under <task>
                       (defaults to the selected servo; repeat to add more
                        servos to the same task)
  drop <task>          Remove a recorded task
  show                 Show everything recorded so far
  yaml                 Print the tasks: YAML block to paste into servo_tasks.yaml

  help                 This message
  q / quit             Exit
""".strip()


def fmt_action(a):
    """Render one recorded action as inline-YAML, matching servo_tasks.yaml."""
    return f"{{servo_id: {a['servo_id']}, angle: {a['angle']}}}"


def print_yaml(tasks):
    """Print the accumulated tasks as a servo_tasks.yaml-compatible block."""
    if not tasks:
        print("(nothing recorded yet — use `add <task>` first)")
        return
    print("\n# ---- paste into config/servo_tasks.yaml under `tasks:` ----")
    print("tasks:")
    for name, actions in tasks.items():
        print(f"  {name}:")
        for a in actions:
            print(f"    - {fmt_action(a)}")
    print("# -----------------------------------------------------------\n")


def read_angle(servo):
    """Read back absolute position, printing a friendly line. Returns deg or None."""
    pos = servo.position()
    if pos is None:
        print(f"  servo {servo.id}: position read FAILED")
        return None
    print(f"  servo {servo.id}: at {pos:.1f} deg")
    return pos


def do_move(servo, abs_deg, settle=0.8):
    """Move to an absolute angle, let it settle, and read back."""
    abs_deg = max(0.0, min(360.0, abs_deg))
    print(f"  -> moving servo {servo.id} to {abs_deg:.1f} deg abs...")
    if not servo.move(abs_deg):
        print("  move NOT acknowledged (no response / outside position limits?)")
    time.sleep(settle)
    return read_angle(servo)


def record(tasks, servo, name):
    """Record the servo's current read-back angle under task `name`."""
    pos = servo.position()
    if pos is None:
        print(f"  cannot record — servo {servo.id} position read failed")
        return
    action = {"servo_id": servo.id, "angle": round(pos, 1)}
    tasks.setdefault(name, [])
    # Replace any existing action for this servo in this task
    tasks[name] = [a for a in tasks[name] if a["servo_id"] != servo.id]
    tasks[name].append(action)
    print(f"  recorded {name}: {fmt_action(action)}")


def scan_and_show(bus, max_id=20):
    """Scan the bus and print each servo's info + telemetry. Returns id list."""
    print(f"Scanning bus (IDs 1-{max_id})...")
    found = bus.scan(max_id)
    if not found:
        print("No servos found. Check wiring / power / 50% sample-point.")
        return found
    print(f"Found {len(found)} servo(s): {found}")
    for sid in found:
        s = bus.servo(sid)
        info = s.info()
        if info:
            print(f"  {info}")
        t = s.telemetry()
        if t:
            print(f"  {t}")
    return found


def repl(interface):
    print(f"=== Servo setpoint finder on {interface} ===")
    tasks = {}   # task_name -> [action, ...]
    with HitecBus(interface) as bus:
        found = scan_and_show(bus)
        current = bus.servo(found[0]) if found else None
        if current:
            print(f"\nSelected servo {current.id}. Type `help` for commands.")
        else:
            print("\nNo servo selected. Fix wiring, then `scan` and `use <id>`.")

        while True:
            try:
                line = input(f"[servo {current.id if current else '?'}]> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue

            parts = line.split()
            cmd, args = parts[0].lower(), parts[1:]

            try:
                if cmd in ("q", "quit", "exit"):
                    break
                elif cmd == "help":
                    print(HELP)
                elif cmd == "scan":
                    found = scan_and_show(bus)
                elif cmd == "use":
                    current = bus.servo(int(args[0]))
                    print(f"  selected servo {current.id}")
                    read_angle(current)
                elif current is None:
                    print("  no servo selected — `use <id>` first")
                elif cmd == "a":
                    do_move(current, float(args[0]))
                elif cmd == "r":
                    do_move(current, 180.0 + float(args[0]))
                elif cmd in ("+", "-"):
                    pos = current.position()
                    if pos is None:
                        print("  position read failed — cannot nudge")
                    else:
                        delta = float(args[0]) * (1 if cmd == "+" else -1)
                        do_move(current, pos + delta)
                elif cmd == "p":
                    read_angle(current)
                elif cmd == "t":
                    t = current.telemetry()
                    print(f"  {t}" if t else "  telemetry read failed")
                elif cmd == "unlock":
                    ok = current.set_limits_relative(180)
                    # print("  limits -> 30-330 deg (+-150)" if ok else "  failed to set limits")
                elif cmd == "limits":
                    ok = current.set_limits(float(args[0]), float(args[1]))
                    print(f"  limits -> {args[0]}-{args[1]} deg" if ok else "  failed")
                elif cmd == "torque":
                    ok = current.set_torque_limit(float(args[0]))
                    print(f"  torque limit -> {args[0]}%" if ok else "  failed")
                elif cmd == "save":
                    current.save()
                    print("  saved to flash (survives power cycle)")
                elif cmd == "add":
                    servo = bus.servo(int(args[1])) if len(args) > 1 else current
                    record(tasks, servo, args[0])
                elif cmd == "drop":
                    if tasks.pop(args[0], None) is not None:
                        print(f"  dropped {args[0]}")
                    else:
                        print(f"  no such task: {args[0]}")
                elif cmd == "show":
                    print_yaml(tasks)
                elif cmd == "yaml":
                    print_yaml(tasks)
                else:
                    print(f"  unknown command: {cmd} (try `help`)")
            except (IndexError, ValueError):
                print(f"  bad arguments for `{cmd}` — see `help`")

        # On exit, dump whatever was recorded so nothing is lost.
        if tasks:
            print("Recorded setpoints:")
            print_yaml(tasks)
        print("Bye.")


def main():
    interface = sys.argv[1] if len(sys.argv) > 1 else INTERFACE
    repl(interface)


if __name__ == "__main__":
    main()
