import serial
import time

PORT = "/dev/ttyTHS1"
BAUD = 921600

CONFIG_COMMANDS = [
    "=config,1",
    "=outputfmt,B",
    "=baud,921600",
    "=dr,100",
    "=angunits,RAD",
    "=angfmt,RATE",
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

ser = serial.Serial(PORT, BAUD, timeout=0)
configure(ser)

last = -1e99
while True:
    data = ser.read(ser.in_waiting or 40)
    if data:
        t = time.time()
        print(f"[{1000*(t-last):.2f} ms] Received {len(data)} bytes: {data.hex()}")
        last = t
