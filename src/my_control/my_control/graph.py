import pandas as pd
import matplotlib.pyplot as plt

def analyze_pid_log(file_name):
    # อ่านไฟล์ CSV
    try:
        data = pd.read_csv(file_name)
    except FileNotFoundError:
        print(f"Error: ไม่พบไฟล์ {file_name}")
        return

    # สร้างกราฟ
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    
    # คำนวณเวลาเริ่มต้นให้เป็น 0 (ใช้ .values เพื่อแก้ ValueError)
    time_x = (data['Timestamp'] - data['Timestamp'].iloc[0]).values
    
    # กราฟล้อซ้าย (เติม .values ทุกจุดที่มีปัญหา)
    ax1.plot(time_x, data['Set_L'].values, 'r--', label='Setpoint L')
    ax1.plot(time_x, data['Act_L'].values, 'b-', label='Actual L')
    ax1.set_title(f'Left Wheel Analysis: {file_name}')
    ax1.legend()
    ax1.set_ylabel('Velocity (m/s)')
    ax1.grid(True)

    # กราฟล้อขวา
    ax2.plot(time_x, data['Set_R'].values, 'r--', label='Setpoint R')
    ax2.plot(time_x, data['Act_R'].values, 'g-', label='Actual R')
    ax2.set_title('Right Wheel Analysis')
    ax2.legend()
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

# เรียกใช้งาน
analyze_pid_log('pid_log_20260302_231206.csv')