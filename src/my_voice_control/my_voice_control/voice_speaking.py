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
        
        # --- Publishers ---
        # ส่งสัญญาณไปปลดล็อกไมค์ที่ VAD Node
        self.pub_listen = self.create_publisher(Bool, '/listen', 10)
        
        # --- Subscriptions ---
        # รับข้อความที่ต้องการให้หุ่นยนต์พูด
        self.sub_tts = self.create_subscription(
            String, 
            '/speaking_request', 
            self.tts_callback, 
            10)
        
        self.get_logger().info("📢 TTS Node Ready. Waiting for /speaking_request...")

    def tts_callback(self, msg):
        text_to_speak = msg.data
        if not text_to_speak:
            return

        self.get_logger().info(f"🎙️ Robot is speaking: {text_to_speak}")
        
        temp_file = "temp_speech.mp3"
        try:
            # 1. แปลงข้อความเปนเสียง (gTTS)
            tts = gTTS(text=text_to_speak, lang='th')
            tts.save(temp_file)
            
            # 2. เล่นเสียง (ขั้นตอนนี้จะ Block จนกว่าจะพูดจบ)
            playsound.playsound(temp_file)
            
        except Exception as e:
            self.get_logger().error(f"❌ TTS Error: {e}")
        finally:
            # ลบไฟล์ชั่วคราว
            if os.path.exists(temp_file):
                os.remove(temp_file)
            
            # --- [หัวใจสำคัญ] ---
            # เมื่อพูดจบแล้ว (โปรแกรมหลุดจาก playsound) ให้ส่งสัญญาณปลดล็อกไมค์ทันที
            self.get_logger().info("✅ Speaking finished. Unlocking microphone...")
            
            unlock_msg = Bool()
            unlock_msg.data = True
            self.pub_listen.publish(unlock_msg)

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