# tauv_servo

Task-driven ROS 2 driver **and** standalone Python library for Hitec
MDB961WP-CAN servos over CAN bus.

- **`servo_driver`** (ROS 2 node) — publish a task name as a string, the node
  drives the mapped servo(s) to the mapped position(s). See [ROS 2 driver](#ros-2-driver).
- **`hitec_servo.py`** (library) — the underlying CAN protocol implementation,
  usable on its own. See [Library usage](#usage).

Built from the Hitec CAN Protocol Manual (Rev 2.03_EN). Every register address in the code is annotated with its manual section and page number. No existing library for this protocol existed.

## ROS 2 driver

The `servo_driver` node subscribes to a `std_msgs/String` **task** topic and
executes the named task from `config/servo_tasks.yaml`. It publishes a
`std_msgs/String` status: `running_<task>` when the task starts, `done_<task>`
once every servo has been read back and **verified** to have reached its target
angle, and `error_<task>` on any failure (unknown task, no ack, or angle not
reached). On startup it also **scans the CAN bus** and logs every servo it finds
plus a telemetry snapshot for each.

```bash
# Build
colcon build --packages-select tauv_servo
source install/setup.bash

# Bring up the CAN interface (once per boot — 50% sample point is mandatory)
sudo ip link set can0 type can bitrate 1000000 sample-point 0.500
sudo ip link set can0 up

# Run the node
ros2 run tauv_servo servo_driver

# Trigger a task (name matched case-insensitively)
ros2 topic pub --once /servo/task std_msgs/String "{data: 'drop_marker'}"

# Watch status  ->  running_drop_marker  then  done_drop_marker (or error_drop_marker)
ros2 topic echo /servo/status
```

### Defining tasks

Tasks live in [`config/servo_tasks.yaml`](config/servo_tasks.yaml). Each task is
an ordered list of servo actions. Each action needs `servo_id` and **exactly one**
position key — the three differ only in what the angle is measured *from*:

| Key | Measured from | Use when |
|-----|---------------|----------|
| `position_deg` | absolute 0–360° (180 = center) | you know the absolute angle |
| `position_relative` | the servo's center (180°) | symmetric travel about center |
| `position_startup` | **where the servo was when the node started** | a mechanism whose safe travel is defined around its resting position |

```yaml
tasks:
  nudge_up:
    - {servo_id: 1, position_startup: +10, torque_limit: 25}  # 10 deg from rest
  home:
    - {servo_id: 1, position_startup: 0}        # back to the startup angle
  open:
    - {servo_id: 2, position_deg: 90}           # absolute
```

The **startup zero** is captured once, during the init bus scan, and never
updated — so `position_startup: 0` always returns to where the mechanism *began*,
not to wherever the last task left it. The servo holds its physical angle across
a reboot (absolute encoder, no homing, no snap to center), so the startup zero is
the mechanism's true resting angle.

Malformed actions are logged and skipped at startup.

### Per-servo limits (the safety layer)

The optional `servos:` section sets each servo's position limits and torque cap
during init. These limits are enforced **by the servo itself** — it physically
clamps any target beyond them — so a bug in this node cannot drive a mechanism
past its window.

```yaml
servos:
  1:
    limit_mode: startup   # window follows wherever the servo powered up
    limit_deg: 20         # hard +-20 deg guard rail around the startup angle
    torque_limit: 25
  2:
    limit_mode: center    # window centered on the servo's own center (180)
    limit_deg: 150        # 30-330 deg
    torque_limit: 25
```

`limit_mode: startup` is the one to use for a constrained mechanism. Servos not
listed fall back to the global `position_limit_deg` parameter. `limit_deg: 0`
leaves the servo's existing limits untouched.

Limits are written to RAM and are **lost on power cycle** unless `save_limits` is
set — the node re-applies them on every startup, which is the intended design for
a `startup`-mode window (it has to follow wherever the mechanism actually rests).

### Finding setpoints

Don't guess the angles — use the interactive **`find_setpoints`** tool. It jogs
servos to positions, reads back where they actually landed, and prints a
ready-to-paste `tasks:` block. It never writes the YAML itself.

```bash
ros2 run tauv_servo find_setpoints          # or: python3 find_setpoints.py [interface]
```

```
[servo 1]> unlock          # open full ±150° range so moves aren't ignored
[servo 1]> a 250           # move to 250° absolute, reads back actual angle
[servo 1]> + 5             # nudge +5° until the mechanism looks right
[servo 1]> add drop_marker # record servo 1's current angle under "drop_marker"
[servo 1]> use 2           # switch to servo 2
[servo 2]> a 90
[servo 2]> add drop_marker # add servo 2 to the same task
[servo 2]> yaml            # print the tasks: block to paste into servo_tasks.yaml
```

Type `help` in the tool for the full command list (`r`/`+`/`-` jogging, `t`
telemetry, `limits`, `torque`, `rel` to record a relative offset, `save`, etc.).

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `interface` | `can0` | SocketCAN interface |
| `bitrate` | `1000000` | CAN bitrate |
| `tasks_config` | installed `config/servo_tasks.yaml` | Task map path |
| `command_topic` | `servo/task` | Task name input topic |
| `status_topic` | `servo/status` | Status output topic |
| `torque_limit` | `0.0` | Global torque cap % (0 = leave servo default). Overridden by `servos:` |
| `startup_task` | `` | Task to run once on boot (e.g. `center`) |
| `scan_max_id` | `20` | Highest servo ID probed during the startup bus scan |
| `position_limit_deg` | `150.0` | Fallback ±limit (deg) for servos **not** in `servos:`; `0` = leave as-is |
| `hold_on_startup` | `true` | Command each servo to hold its as-found angle at init (see below) |
| `save_limits` | `false` | Persist the limits to servo flash (survives power cycle) |
| `angle_tolerance` | `3.0` | Readback verify window in degrees |
| `verify_timeout` | `2.0` | Max seconds to wait for a servo to reach its target |
| `verify_poll` | `0.1` | Position readback interval while verifying |

> **Note:** a target outside the servo's position limits is **clamped** — the
> servo drives to the limit and stops (factory default ±60°). On init the node
> unlocks the full **±150°** range (`position_limit_deg`) on every servo it
> finds, so task targets in 30–330° work out of the box. This is applied in RAM
> each startup; set `save_limits: true` to also persist it to the servo's flash.
> The node also refuses to command a target outside the servo's limits, since
> clamping would otherwise move the mechanism somewhere you didn't ask for; such
> a task reports `error_<task>`.

> **`hold_on_startup`:** the servo powers up with its motor **idle** (torque 0%)
> and only starts driving once it receives a position command — so a loaded
> mechanism can sag or back-drive until something engages it. With
> `hold_on_startup` (default `true`) the node reads each servo's as-found angle
> and immediately commands it to hold there. This is a zero-distance move: it
> engages holding torque **without moving the output**. The torque cap is applied
> *before* this, so the motor never engages at full power.

## Library

## Setup

```bash
pip install python-can

# CAN interface (run after each boot)
sudo ip link set can0 type can bitrate 1000000 sample-point 0.500
sudo ip link set can0 up
```

The **50% sample point** is mandatory — mismatch causes silent failure.

## First-time servo config

Connect **one servo at a time**, then:

```bash
python3 test_hitec.py setup 1    # sets ID, mode, baud, limits
# power cycle servo, repeat for next servo with ID 2, etc.
```

This sets: CAN 2.0A mode, 1Mbps, 50% sample point, servo mode, and **±150° position limits**.

## Usage

```python
from hitec_servo import HitecBus

with HitecBus("can0") as bus:
    s1 = bus.servo(1)
    s2 = bus.servo(2)

    # Absolute positioning (0-360°, 180° = center)
    s1.move(90)
    s2.move(270)

    # Relative positioning (±150° from center)
    s1.move_relative(-90)     # = 90° absolute
    s1.move_relative(+90)     # = 270° absolute
    s1.move_relative(0)       # = center (180°)

    # Read position both ways
    print(s1.position())           # 180.0 (absolute)
    print(s1.position_relative())  # 0.0 (from center)

    # Telemetry
    print(s1.telemetry())

    # Torque limit (0-100%) — IMPORTANT for safety
    s1.set_torque_limit(50)

    # Read current limits
    print(s1.get_limits())         # (30.0, 330.0)
```

## Position limits — critical detail

A move command outside the servo's configured limits is **clamped**: the servo
drives to the limit and stops there. It acknowledges the command and raises no
error flag — so an out-of-range target still produces **real motion**, which can
drive a mechanism into a hard stop. Verified on MDB961WP hardware: with limits
set to 150°-210°, commanding 300° moved the servo to 209.8°, and commanding 60°
moved it to 150.2°.

The MDB961WP defaults to ±60° (120°-240°). The `setup_servo()` function sets
±150° (30°-330°) automatically. You can also set them manually:

```python
s.set_limits(30, 330)        # absolute degrees
s.set_limits_relative(150)   # same thing, symmetric
s.save()                     # persist across power cycles
# power cycle servo
```

## Torque limiting

```python
s.set_torque_limit(50.0)   # cap at 50% motor output
```

Hitec warns: these servos have **no internal overheating protection**. Stalling at full torque for >5-10 seconds can damage them. Always set an appropriate limit.

## Test script

```bash
python3 test_hitec.py scan           # find servos, show limits
python3 test_hitec.py move 1         # test absolute moves
python3 test_hitec.py relative 1     # test ±offset moves
python3 test_hitec.py telemetry 1    # read all sensors
python3 test_hitec.py torque 1       # test torque limiting
python3 test_hitec.py monitor 1      # live telemetry
python3 test_hitec.py demo 1,2       # random moves on servos 1 and 2
```

## Files

- `tauv_servo/servo_driver.py` — ROS 2 node (task string → servo moves)
- `tauv_servo/hitec_servo.py` — CAN protocol library (single file)
- `tauv_servo/find_setpoints.py` — Interactive tool to discover task setpoints for the YAML
- `tauv_servo/test_hitec.py` — CLI tests and examples
- `config/servo_tasks.yaml` — Task → servo action map
- `package.xml` / `setup.py` / `setup.cfg` — ament_python package files
- `README.md` — This file

## Protocol docs

https://www.hiteccs.com/public/uploads/ckeditor/655bb53e9fcca1700508990.pdf
