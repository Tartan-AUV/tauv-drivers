import serial
import time

ser = serial.Serial("/dev/ttyTHS1", 921600, timeout=0)
start = time.time()
last = -1e99
while True:
    data = ser.read(ser.in_waiting or 38)
    if data:
        t = time.time()
        print(f"[{1000*(t-last):.2f}] Received: {data}")
        last = t
