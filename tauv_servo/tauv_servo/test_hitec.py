#!/usr/bin/env python3
"""Test and examples for hitec_servo.py. Run: python3 test_hitec.py <command>"""

import time, sys, random

# Works both as a package module (ros2 run / colcon) and when run directly
# from this directory (python3 test_hitec.py ...).
try:
    from tauv_servo.hitec_servo import HitecBus, setup_servo
except ImportError:
    from hitec_servo import HitecBus, setup_servo

INTERFACE = "can0"

def test_scan():
    print("=== Scanning ===")
    with HitecBus(INTERFACE) as bus:
        found = bus.scan(10)
        if found:
            for sid in found:
                s = bus.servo(sid)
                print(s.info())
                limits = s.get_limits()
                if limits:
                    print(f"    Limits: {limits[0]:.0f}°-{limits[1]:.0f}° "
                          f"(±{(limits[1]-limits[0])/2:.0f}° from center)")
        else:
            s = bus.broadcast()
            if s.ping():
                print("Servo responds to broadcast. Run: python3 test_hitec.py setup 1")
            else:
                print("No servos found. Check wiring/power/sample-point.")

def test_move(sid: int):
    print(f"=== Move Test (servo {sid}) ===")
    with HitecBus(INTERFACE) as bus:
        s = bus.servo(sid)
        if not s.ping():
            print(f"Servo {sid} not responding!"); return

        limits = s.get_limits()
        print(f"Position limits: {limits}" if limits else "Could not read limits")

        for angle in [180, 120, 240, 90, 270, 180]:
            print(f"  -> {angle}°...", end=" ", flush=True)
            s.move(angle)
            time.sleep(1.0)
            pos = s.position()
            print(f"at {pos:.1f}°" if pos else "read failed")

def test_relative(sid: int):
    print(f"=== Relative Move Test (servo {sid}) ===")
    with HitecBus(INTERFACE) as bus:
        s = bus.servo(sid)
        if not s.ping():
            print(f"Servo {sid} not responding!"); return

        for offset in [0, -45, +45, -90, +90, -130, +130, 0]:
            print(f"  -> {offset:+.0f}°...", end=" ", flush=True)
            s.move_relative(offset)
            time.sleep(1.0)
            rel = s.position_relative()
            print(f"at {rel:+.1f}°" if rel is not None else "read failed")

def test_telemetry(sid: int):
    print(f"=== Telemetry (servo {sid}) ===")
    with HitecBus(INTERFACE) as bus:
        s = bus.servo(sid)
        t = s.telemetry()
        print(t if t else "Failed to read telemetry!")

def test_torque(sid: int):
    print(f"=== Torque Limit Test (servo {sid}) ===")
    with HitecBus(INTERFACE) as bus:
        s = bus.servo(sid)
        print(f"Current limit: {s.get_torque_limit():.1f}%")

        s.set_torque_limit(30.0)
        print(f"Set to 30%: {s.get_torque_limit():.1f}%")
        s.move(120); time.sleep(1.5)
        print(f"  Position: {s.position():.1f}°, Torque: {s.torque():.1f}%")

        s.set_torque_limit(100.0)
        print(f"Restored to 100%: {s.get_torque_limit():.1f}%")

def test_monitor(sid: int, dur: float = 5.0):
    print(f"=== Monitor (servo {sid}, {dur}s) ===")
    with HitecBus(INTERFACE) as bus:
        s = bus.servo(sid)
        start = time.monotonic()
        while (time.monotonic() - start) < dur:
            t = s.telemetry()
            if t: print(f"\r{t}", end="", flush=True)
            time.sleep(0.1)
        print()

def test_demo(sids: list):
    print(f"=== Random Move Demo (servos {sids}) ===")
    with HitecBus(INTERFACE) as bus:
        servos = [bus.servo(sid) for sid in sids]
        for s in servos:
            if not s.ping():
                print(f"Servo {s.id} not responding!"); return
            limits = s.get_limits()
            print(f"Servo {s.id}: limits={limits}")
            s.set_torque_limit(70.0)

        print("Moving randomly (Ctrl+C to stop)\n")
        try:
            while True:
                for s in servos:
                    offset = random.uniform(-120, 120)
                    s.move_relative(offset)
                    print(f"  Servo {s.id} -> {offset:+.1f}°")
                time.sleep(1.5)
                for s in servos:
                    t = s.telemetry()
                    if t: print(f"  {t}")
                print()
        except KeyboardInterrupt:
            print("\nCentering...")
            bus.move_all(180)
            print("Done.")

if __name__ == "__main__":
    cmds = {
        "scan":      "Scan for servos",
        "move ID":   "Test absolute movement",
        "relative ID": "Test relative (±) movement",
        "telemetry ID": "Read all sensors",
        "torque ID": "Test torque limiting",
        "monitor ID": "Live telemetry",
        "demo ID,ID": "Random moves (e.g. demo 1,2)",
        "setup ID":  "Configure new servo",
    }

    if len(sys.argv) < 2:
        print("Usage: python3 test_hitec.py <command> [args]\n")
        for c, d in cmds.items():
            print(f"  {c:16s} {d}")
        sys.exit(0)

    cmd = sys.argv[1]
    if cmd == "scan":       test_scan()
    elif cmd == "move":     test_move(int(sys.argv[2]))
    elif cmd == "relative": test_relative(int(sys.argv[2]))
    elif cmd == "telemetry":test_telemetry(int(sys.argv[2]))
    elif cmd == "torque":   test_torque(int(sys.argv[2]))
    elif cmd == "monitor":  test_monitor(int(sys.argv[2]))
    elif cmd == "demo":     test_demo([int(x) for x in sys.argv[2].split(",")])
    elif cmd == "setup":    setup_servo(INTERFACE, int(sys.argv[2]))
    else: print(f"Unknown: {cmd}")
