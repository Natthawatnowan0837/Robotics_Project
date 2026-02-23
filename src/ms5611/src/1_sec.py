import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# ========================= CONFIG =========================
PORT = "/dev/ttyUSB0"   # แก้ตามพอร์ตจริง
BAUD = 115200
DURATION_SEC = 1.0       # เก็บข้อมูล 1 วินาที
PRESS_UNIT = "hPa"       # บอร์ดส่งเป็น hPa แล้ว
# ===========================================================


def parse_line(line):
    """
    แปลงข้อความจาก Serial เช่น:
    CNT DUR RES TEMP PRES
    123860 3124 0 28.19 970.53
    """
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    parts = line.split()
    if len(parts) < 5:
        return None
    try:
        return {
            "cnt": int(parts[0]),
            "dur": int(parts[1]),
            "res": int(parts[2]),
            "temp": float(parts[3]),
            "pres": float(parts[4])
        }
    except ValueError:
        return None


def read_1s_burst(ser, duration_sec=1.0):
    t_list, pres_list, temp_list = [], [], []
    ser.reset_input_buffer()
    start = time.monotonic()
    while time.monotonic() - start < duration_sec:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode(errors="ignore")
        pkt = parse_line(line)
        if pkt:
            t_list.append(time.monotonic() - start)
            pres_list.append(pkt["pres"])
            temp_list.append(pkt["temp"])
    return np.array(t_list), np.array(pres_list), np.array(temp_list)


def moving_average(x, w=5):
    if len(x) < w:
        return x
    return np.convolve(x, np.ones(w)/w, mode='valid')


def plot_pressure(t, pres):
    plt.figure("Pressure (1s burst)", figsize=(9, 4))
    plt.plot(t, pres, label="raw")
    ma = moving_average(pres, w=5)
    plt.plot(t[:len(ma)], ma, label="MA(5)")
    plt.title("Pressure (1s burst)")
    plt.xlabel("Time (s)")
    plt.ylabel(f"Pressure ({PRESS_UNIT})")
    plt.grid(True, alpha=0.3)
    plt.legend()


def plot_temperature(t, temp):
    plt.figure("Temperature (1s burst)", figsize=(9, 4))
    plt.plot(t, temp, label="raw", color='tab:red')
    ma = moving_average(temp, w=5)
    plt.plot(t[:len(ma)], ma, label="MA(5)", color='tab:orange')
    plt.title("Temperature (1s burst)")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (°C)")
    plt.grid(True, alpha=0.3)
    plt.legend()


def summarize(t, pres, temp):
    if len(pres) == 0:
        print("❌ No data captured.")
        return
    print("\n--- SUMMARY ---")
    print(f"Samples: {len(pres)}")
    print(f"Pressure mean={np.mean(pres):.2f} {PRESS_UNIT}, std={np.std(pres):.4f}")
    print(f"Temperature mean={np.mean(temp):.2f} °C, std={np.std(temp):.4f}")
    dt = t[-1] - t[0]
    if dt > 0:
        print(f"Sampling rate ≈ {len(pres)/dt:.1f} Hz")


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(2)
    print(f"Connected to {PORT}")

    try:
        while True:
            input("กด Enter เพื่ออ่านค่าจากเซนเซอร์ 1 วินาที...")
            t, pres, temp = read_1s_burst(ser, DURATION_SEC)
            summarize(t, pres, temp)
            plot_pressure(t, pres)
            plot_temperature(t, temp)
            plt.show()
    except KeyboardInterrupt:
        print("\nหยุดโปรแกรมด้วย Ctrl+C")
    finally:
        ser.close()
        print("Serial ปิดแล้ว")


if __name__ == "__main__":
    main()
