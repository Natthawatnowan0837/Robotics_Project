#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from gtts import gTTS
import playsound
import os
import time

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        
        # 1. ตัวรับคำสั่งว่า "ให้พูดคำว่าอะไร"
        self.sub_tts = self.create_subscription(
            String, 
            '/speaking_request', 
            self.tts_callback, # ชื่อต้องตรงกับฟังก์ชันข้างล่าง
            10
        )
        
        # 2. ตัวบอกโหนดอื่นว่า "ตอนนี้กำลังพูดอยู่ห้ามเปิดไมค์นะ"
        self.pub_status = self.create_publisher(Bool, '/robot_is_speaking', 10)
        
        self.get_logger().info("📢 TTS Node is Ready (Waiting for /speaking_request)")

    def tts_callback(self, msg):
        """ฟังก์ชันหลัก: รับข้อความมาแปลงเป็นเสียงแล้วเล่นออกลำโพง"""
        text_to_speak = msg.data
        if not text_to_speak:
            return

        self.get_logger().info(f"🎙️ Speaking: {text_to_speak}")

        # --- ขั้นตอนการทำงาน ---
        
        # A. ส่งสถานะบอกว่า "เริ่มพูดแล้วนะ" (ไมค์ฝั่งรับเสียงจะล็อคทันที)
        status_msg = Bool()
        status_msg.data = True
        self.pub_status.publish(status_msg)

        try:
            # B. ใช้ gTTS สร้างไฟล์เสียงชั่วคราว
            tts = gTTS(text=text_to_speak, lang='th')
            temp_file = "temp_speech.mp3"
            tts.save(temp_file)
            
            # C. เล่นเสียง (โปรแกรมจะหยุดรอจนกว่าเสียงจะจบที่บรรทัดนี้)
            playsound.playsound(temp_file)
            
            # D. ลบไฟล์ทิ้ง
            if os.path.exists(temp_file):
                os.remove(temp_file)
                
        except Exception as e:
            self.get_logger().error(f"❌ Error during TTS: {e}")

        # E. ส่งสถานะบอกว่า "พูดจบแล้ว" (ไมค์ฝั่งรับเสียงจะปลดล็อค)
        status_msg.data = False
        self.pub_status.publish(status_msg)
        self.get_logger().info("✅ Finished speaking.")

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()