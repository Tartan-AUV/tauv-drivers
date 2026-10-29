"""KVH 1775 serial debug tool.

  python3 debug.py           configure the IMU (CONFIG_COMMANDS), then dump the raw stream
  python3 debug.py --query   read back the current settings and exit (changes nothing)

Stop the ROS driver first; only one process can own the port.

Settings read back from our unit on 2026-10-28 with --query (known good):
  AXES       identity (+1,0,0, 0,+1,0, 0,0,+1)
  ROTFMT     RATE          ROTUNITS   RAD
  LINFMT     ACCEL         LINUNITS   METERS
  FILTEN     1             FILTTYPE   G,BUTTER,8,250.00 / A,BUTTER,8,250.00
  DR         500           BAUD       460800
  OUTPUTFMT  B             MSYNC      IMU
  TEMPUNITS  C
Measured the same day with a 180 deg yaw two-position test: gyro scale 1.01,
bias ~-3.7 deg/h horizontal and ~-3.4 deg/h vertical (spec +-0.5 deg/h).
"""
import sys
import serial
import time

PORT = "/dev/ttyTHS1"
BAUD = 460800

CONFIG_COMMANDS = [
    "=config,1",
    "=outputfmt,B",
    "=baud,460800",
    "=dr,500",
    "=rotunits,RAD",   # the manual's command; "=angunits" is rejected as INVALID
    "=rotfmt,RATE",    # factory default is DELTA (angle per sample)
    "=linunits,METERS",
    "=linfmt,ACCEL",
    "=filttype,G,BUTTER",
    "=filttype,A,BUTTER",
    "=config,0",
]

def send_command(ser, cmd):
    ser.write((cmd + "\r\n").encode())
    ser.flush()
    time.sleep(0.1)
    response = ser.read(ser.in_waiting or 1)
    print(f"  >> {cmd}  <- {response.decode(errors='replace').strip()!r}")

def configure(ser):
    print("--- Configuring IMU ---")
    # Drain anything the IMU is already streaming
    time.sleep(0.2)
    ser.reset_input_buffer()
    for cmd in CONFIG_COMMANDS:
        send_command(ser, cmd)
    # After =config,0 the IMU starts streaming; give it a moment to settle
    time.sleep(0.5)
    ser.reset_input_buffer()
    print("--- Configuration done, starting stream ---\n")

QUERY_COMMANDS = [
    "?axes", "?rotfmt", "?rotunits", "?linfmt", "?linunits", "?filten",
    "?filttype,g", "?filttype,a", "?dr", "?baud", "?outputfmt", "?msync", "?tempunits",
]

def query(ser):
    print("--- Current IMU settings ---")
    send_command(ser, "=config,1")
    time.sleep(0.3)
    for cmd in QUERY_COMMANDS:
        ser.reset_input_buffer()
        send_command(ser, cmd)
    send_command(ser, "=config,0")

if "--query" in sys.argv:
    ser = serial.Serial(PORT, BAUD, timeout=0.3)
    query(ser)
    sys.exit(0)

ser = serial.Serial(PORT, BAUD, timeout=0)
configure(ser)

last = -1e99
while True:
    data = ser.read(ser.in_waiting or 40)
    if data:
        t = time.time()
        print(f"[{1000*(t-last):.2f} ms] Received {len(data)} bytes: {data.hex()}")
        last = t
