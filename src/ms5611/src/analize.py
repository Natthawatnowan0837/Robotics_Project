import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
from matplotlib.ticker import MultipleLocator

# ========================= CONFIG =========================
PRESS_UNIT = "hPa"
FILE_PATTERN = "floor*.csv"
SHOW_SEC = 1.0                # แสดงแค่ช่วงเวลา 0–1 s
Y_TICK_STEP = 0.1             # ✅ ความละเอียดแกน Pressure
# ===========================================================

def moving_average(x, w=5):
    if len(x) < w:
        return x
    return np.convolve(x, np.ones(w)/w, mode='valid')

def load_floor_data():
    files = sorted(glob.glob(FILE_PATTERN))
    datasets = []
    for file in files:
        try:
            df = pd.read_csv(file)
            floor_name = os.path.splitext(os.path.basename(file))[0]
            datasets.append((floor_name, df))
            print(f"✅ โหลด {file} ({len(df)} แถว)")
        except Exception as e:
            print(f"⚠️ อ่าน {file} ไม่ได้: {e}")
    return datasets

def compute_time(df):
    if "CNT" not in df.columns:
        df["time(s)"] = np.arange(len(df)) * 0.01
        return df
    df = df.sort_values("CNT")
    dcnt = df["CNT"].diff().median()
    if pd.isna(dcnt) or dcnt == 0:
        dcnt = 1
    df["time(s)"] = (df["CNT"] - df["CNT"].iloc[0]) * (1.0 / dcnt) / len(df)
    df["time(s)"] = df["time(s)"] - df["time(s)"].min()
    return df[df["time(s)"] <= SHOW_SEC]

def plot_pressure_vs_time(datasets):
    plt.figure("Pressure vs Time (0–1 s)", figsize=(10, 5))
    all_pres = []
    for name, df in datasets:
        if "pressure(hPa)" not in df or len(df) == 0:
            continue
        df = compute_time(df)
        pres = df["pressure(hPa)"].values
        time_s = df["time(s)"].values
        pres_ma = moving_average(pres, w=5)
        plt.plot(time_s[:len(pres_ma)], pres_ma, label=f"{name}")
        all_pres.extend(pres_ma)

    plt.title(f"Pressure in sec (First {SHOW_SEC} s)")
    plt.xlabel("Time (s)")
    plt.ylabel(f"Pressure ({PRESS_UNIT})")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # ✅ ตั้งค่าความละเอียดแกน Y ให้เห็นระดับ 0.1 hPa
    if all_pres:
        ymin, ymax = min(all_pres), max(all_pres)
        plt.ylim(ymin - 0.05, ymax + 0.05)  # เผื่อขอบเล็กน้อย
        ax = plt.gca()
        ax.yaxis.set_major_locator(MultipleLocator(Y_TICK_STEP))  # tick ทุก 0.1 hPa
        ax.yaxis.set_minor_locator(MultipleLocator(Y_TICK_STEP / 2))
        ax.grid(which='minor', color='gray', linestyle=':', alpha=0.3)

    plt.tight_layout()
    plt.show()

def main():
    datasets = load_floor_data()
    if not datasets:
        return
    plot_pressure_vs_time(datasets)

if __name__ == "__main__":
    main()
