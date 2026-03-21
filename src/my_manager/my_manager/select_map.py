import subprocess
import sys

def run_launch(floor, direction):
    # รวมคำสั่ง ros2 launch
    command = [
        'ros2', 'launch', 'my_manager', 'launch_localize.launch.py',
        f'floor:={floor}',
        f'db_name:={direction}'
    ]
    
    print(f"\n🚀 กำลังรัน: {' '.join(command)}")
    
    try:
        # ใช้ subprocess.run เพื่อส่งคำสั่งไปยัง Terminal
        # Note: เมื่อกด Ctrl+C จะเป็นการหยุดการทำงานของ Launch file นั้นๆ
        subprocess.run(command)
    except KeyboardInterrupt:
        print("\n🛑 หยุดการทำงานของหุ่นยนต์")
    except Exception as e:
        print(f"❌ เกิดข้อผิดพลาด: {e}")

def main():
    while True:
        print("\n" + "="*30)
        print("   ระบบสลับแผนที่หุ่นยนต์   ")
        print("="*30)
        
        # Step 1: รับข้อมูลชั้น
        floor_input = input("📌 ระบุชั้น (เช่น 1, 2, 3) หรือ 'q' เพื่อออก: ").strip()
        if floor_input.lower() == 'q':
            break
        
        # แปลงเป็น format floorX
        floor = f"floor{floor_input}"

        # Step 2: รับทิศทาง
        print("\n🔹 เลือกทิศทาง:")
        print("1. ไป (go)")
        print("2. กลับ (back)")
        dir_choice = input("👉 เลือก 1 หรือ 2: ").strip()

        if dir_choice == '1':
            direction = 'go'
        elif dir_choice == '2':
            direction = 'back'
        else:
            print("⚠️ เลือกไม่ถูกต้อง กรุณาลองใหม่")
            continue

        # ยืนยันคำสั่ง
        print(f"\n📍 คุณกำลังจะรัน: {floor} | ทิศทาง: {direction}")
        confirm = input("✅ ยืนยันไหม? (y/n): ").lower()
        
        if confirm == 'y':
            run_launch(floor, direction)
        else:
            print("🔄 ยกเลิกการรัน")

if __name__ == '__main__':
    main()