"""stream.py : show the Uno's Raw: stream live, raw counts and converted units.  python stream.py --port COM5"""
import argparse, math, serial, time
ap = argparse.ArgumentParser(); ap.add_argument("--port", default="COM5"); args = ap.parse_args()
ser = serial.Serial(args.port, 115200, timeout=1)
print("COM5 open, Uno restarting ... (Ctrl+C or close the window to stop)")
time.sleep(2.0)
print(" accel (g)                 |a|     gyro (deg/s)              mag (mG)               |B|")
n = 0; t0 = time.time()
try:
    while True:
        line = ser.readline().decode("ascii", "ignore").strip()
        if not line:
            print("(no data for 1 s)"); continue
        if not line.startswith("Raw:"):
            print(line); continue
        try:
            v = [int(x) for x in line[4:].split(",")]
        except ValueError:
            continue
        if len(v) != 9:
            continue
        a = [x / 8192 for x in v[0:3]]; g = [x / 16 for x in v[3:6]]; m = v[6:9]
        n += 1
        rate = n / (time.time() - t0)
        print(f"{a[0]:+7.3f} {a[1]:+7.3f} {a[2]:+7.3f}   {math.sqrt(sum(x*x for x in a)):5.3f}   "
              f"{g[0]:+8.2f} {g[1]:+8.2f} {g[2]:+8.2f}    {m[0]:+6d} {m[1]:+6d} {m[2]:+6d}   "
              f"{math.sqrt(sum(x*x for x in m)):5.0f}   {rate:4.1f} Hz")
except KeyboardInterrupt:
    pass
finally:
    ser.close()
