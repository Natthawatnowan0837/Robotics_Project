import serial
import csv
import time
import os

# --------- CONFIG ---------
PORT = "/dev/ttyUSB0"   # พอร์ตของ Arduino หรือ ESP32
BAUD = 115200
DURATION_SEC = 1        # ระยะเวลาอ่านค่าต่อชั้น (วินาที)
# --------------------------

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        time.sleep(2)
        print(f"✅ Connected to {PORT} at {BAUD} baud")
    except serial.SerialException:
        print(f"❌ ไม่สามารถเชื่อมต่อกับพอร์ต {PORT}")
        return

    try:
        while True:
            floor = input("กรอกชั้น (1-9) หรือ q เพื่อออก: ").strip()
            if floor.lower() == "q":
                break
            if floor not in [str(i) for i in range(1, 10)]:
                print("⚠️ กรุณากรอกตัวเลข 1-9 เท่านั้น")
                continue

            outfile = f"floor{floor}.csv"
            is_new = not os.path.exists(outfile)

            with open(outfile, "a", newline="") as f:
                writer = csv.writer(f)
                if is_new:
                    writer.writerow(["floor", "CNT", "duration(us)", "temperature(C)", "pressure(hPa)"])

                ser.reset_input_buffer()
                print(f"📡 เริ่มบันทึกชั้น {floor} เป็นเวลา {DURATION_SEC} วินาที...")

                start = time.monotonic()
                while time.monotonic() - start < DURATION_SEC:
                    line = ser.readline().decode(errors="ignore").strip()
                    if not line:
                        continue

                    # ข้ามบรรทัดหัวตารางหรือข้อความอื่น ๆ
                    if line.startswith(("CNT", "MS5611", "--", "#")):
                        continue

                    parts = line.split()
                    if len(parts) >= 5:
                        try:
                            cnt = int(parts[0])
                            dur = int(parts[1])
                            res = int(parts[2])
                            temp = float(parts[3])
                            pres = float(parts[4])
                            writer.writerow([floor, cnt, dur, temp, pres])
                            print(f"[{floor}] CNT={cnt}, TEMP={temp:.2f}, PRES={pres:.2f}")
                        except ValueError:
                            continue

            print(f"✅ บันทึกลง '{outfile}' เสร็จเรียบร้อย ({DURATION_SEC} วินาที)\n")

    except KeyboardInterrupt:
        print("\n🛑 หยุดโปรแกรมด้วย Ctrl+C")

    finally:
        if ser.is_open:
            ser.close()
            print("🔒 ปิดการเชื่อมต่อ Serial แล้ว")

if __name__ == "__main__":
    main()
