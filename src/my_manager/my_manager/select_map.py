import subprocess
import sys

def run_launch(mode, floor, direction):
    # เลือกไฟล์ launch ตามโหมดที่เลือก
    launch_file = 'launch_mapping.launch.py' if mode == '1' else 'launch_localize.launch.py'
    mode_name = "MAPPING" if mode == '1' else "LOCALIZATION"

    command = [
        'ros2', 'launch', 'my_manager', launch_file,
        f'floor:={floor}',
        f'db_name:={direction}'
    ]
    
    print(f"\n🚀 [{mode_name}] กำลังรัน: {' '.join(command)}")
    
    try:
        # ใช้ subprocess.run เพื่อส่งคำสั่งไปยัง Terminal
        subprocess.run(command)
    except KeyboardInterrupt:
        print(f"\n🛑 หยุดการทำงานของ {mode_name}")
    except Exception as e:
        print(f"❌ เกิดข้อผิดพลาด: {e}")

def main():
    while True:
        print("\n" + "═"*40)
        print("      🤖 ROBOT MODE SELECTOR      ")
        print("          (jo's switcher)         ")
        print("═"*40)
        
        # --- Step 1: เลือก Mode ---
        print("🔹 STEP 1: เลือกโหมดการทำงาน")
        print("1. สร้างแผนที่ใหม่ (Mapping)")
        print("2. ระบุตำแหน่ง (Localization)")
        mode_choice = input("👉 เลือก 1 หรือ 2 (หรือ 'q' เพื่อออก): ").strip()
        
        if mode_choice.lower() == 'q': break
        if mode_choice not in ['1', '2']:
            print("⚠️ เลือกโหมดไม่ถูกต้อง!")
            continue

        # --- Step 2: ระบุชั้น ---
        floor_input = input("\n📌 STEP 2: ระบุชั้น (เช่น 1, 2, 3): ").strip()
        floor = f"floor{floor_input}"

        # --- Step 3: เลือกทิศทาง ---
        print("\n🔹 STEP 3: เลือกทิศทาง (Database Name)")
        print("1. ไป (go)")
        print("2. กลับ (back)")
        dir_choice = input("👉 เลือก 1 หรือ 2: ").strip()

        if dir_choice == '1':
            direction = 'go'
        elif dir_choice == '2':
            direction = 'back'
        else:
            print("⚠️ เลือกทิศทางไม่ถูกต้อง!")
            continue

        # --- ยืนยันและรัน ---
        mode_txt = "MAPPING" if mode_choice == '1' else "LOCALIZATION"
        print(f"\n📋 สรุปรายการ: [{mode_txt}] | {floor} | {direction}")
        confirm = input("✅ ยืนยันการรันโหมดนี้หรือไม่? (y/n): ").lower()
        
        if confirm == 'y':
            run_launch(mode_choice, floor, direction)
        else:
            print("🔄 ยกเลิกและกลับสู่หน้าเมนูหลัก")

if __name__ == '__main__':
    main()