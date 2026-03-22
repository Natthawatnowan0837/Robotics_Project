#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import torch
import os
import json
import time
import soundfile as sf
import playsound
from transformers import WhisperProcessor, WhisperForConditionalGeneration
from gtts import gTTS
from thefuzz import fuzz
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool, Float32MultiArray # เพิ่มตัวนี้

STATE_WAITING_WAKEWORD = 0
STATE_LISTENING_COMMAND = 1

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('wave_to_text_node')
        self.is_processing = False
        
        # --- [NEW] ระบบ Topic Communication ---
        # 1. รอรับสัญญาณจาก VAD ว่า "อัดเสร็จแล้ว" (True)
        self.sub_voice_state = self.create_subscription(Bool, '/voice_state', self.voice_state_callback, 10)
        
        # 2. ส่งสัญญาณสั่ง VAD ว่า "ให้เริ่มฟัง" หรือ "หยุดฟัง" (แทนการใช้ Service)
        self.pub_listen = self.create_publisher(Bool, '/listen', 10)
        
        # Publishers อื่นๆ
        self.pub_target = self.create_publisher(Float32MultiArray, '/robot_target', 10)
        self.pub_speaking_status = self.create_publisher(Bool, '/robot_is_speaking', 10)

        # Whisper Model Loading
        self.MODEL_NAME = "biodatlab/whisper-th-small-combined"
        self.get_logger().info(f"⏳ Loading Whisper Model...")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.processor = WhisperProcessor.from_pretrained(self.MODEL_NAME)
        self.model = WhisperForConditionalGeneration.from_pretrained(self.MODEL_NAME).to(self.device)
        self.get_logger().info(f"✅ Whisper Ready on {self.device}")

        # Configuration & State
        self.load_commands()
        self.current_state = STATE_WAITING_WAKEWORD
        self.last_interaction_time = time.time()
        self.MATCH_THRESHOLD = 85 
        self.TIMEOUT_SECONDS = 15.0
        self.command_retry_count = 0
        self.AUDIO_FILE = "my_command.wav" # ชื่อไฟล์ที่ตกลงกับ VAD ไว้

        # เริ่มต้น: ทักทายและสั่งให้ VAD เริ่มฟัง
        self.greeting_timer = self.create_timer(2.0, self.say_greeting)
        self.timer = self.create_timer(1.0, self.check_timeout)

    def say_greeting(self):
        self.speak("สวัสดีครับ น้องพี พร้อมช่วยครับ")
        self.greeting_timer.cancel()

    def voice_state_callback(self, msg):
        """เมื่อได้รับสัญญาณ True จาก VAD หมายถึงมีไฟล์อัดเสร็จแล้ว"""
        if msg.data and not self.is_processing:
            if os.path.exists(self.AUDIO_FILE):
                self.process_audio(self.AUDIO_FILE)
            else:
                self.get_logger().warn(f"⚠️ สัญญาณมาแต่หาไฟล์ {self.AUDIO_FILE} ไม่เจอ")
                self.send_listen_signal(True) # สั่งให้ฟังใหม่

    def process_audio(self, audio_file):
        self.is_processing = True
        # เมื่อเริ่มประมวลผล สั่งปิดไมค์ VAD ทันทีเพื่อความชัวร์
        self.send_listen_signal(False)
        
        try:
            audio, sample_rate = sf.read(audio_file)
            input_features = self.processor(audio, sampling_rate=sample_rate, return_tensors="pt").input_features.to(self.device)

            with torch.no_grad():
                predicted_ids = self.model.generate(input_features, language='th', task='transcribe')
            
            raw_text = self.processor.batch_decode(predicted_ids, skip_special_tokens=True)[0].strip()
            
            if not raw_text:
                self.get_logger().info("... Silence ...")
                self.send_listen_signal(True)
                return

            print(f"\n🗣️ User said: {raw_text}")

            if self.current_state == STATE_WAITING_WAKEWORD:
                self.handle_wake_word(raw_text)
            else:
                self.handle_command(raw_text)

        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")
            self.send_listen_signal(True)
        finally:
            if os.path.exists(audio_file):
                os.remove(audio_file)
            self.is_processing = False

    def send_listen_signal(self, status: bool):
        """ส่งสัญญาณเปิด/ปิดไมค์ไปที่ VAD"""
        msg = Bool()
        msg.data = status
        self.pub_listen.publish(msg)

    def handle_wake_word(self, text):
        is_awake = any(kw in text for kw in self.wake_words)
        if not is_awake:
            for kw in self.wake_words:
                if len(kw) >= 3 and fuzz.partial_ratio(kw, text) >= self.MATCH_THRESHOLD:
                    is_awake = True
                    break

        if is_awake:
            self.current_state = STATE_LISTENING_COMMAND
            self.last_interaction_time = time.time() # รีเซ็ตจุดเริ่มต้นของ Timeout
            self.command_retry_count = 0
            
            # เช็คคำสั่งพร้อม Wake word
            action = self.find_action(text)
            room = self.find_room(text)
            if action and room:
                self.handle_command(text)
            else:
                self.speak("ครับผม ว่าไงครับ")
                # เมื่อพูดจบ VAD จะถูกสั่ง listen=True ทำให้เริ่มนับ 8.0 วินาทีใหม่ของฝั่ง VAD
        else:
            self.send_listen_signal(True) # ไม่ใช่ wake word ให้เริ่มฟังรอบใหม่ทันที
    def handle_command(self, text):
            action = self.find_action(text)
            room = self.find_room(text)

            if action and room:
                room_info = self.rooms_dict[room]
                position = room_info.get("position") # สมมติว่าใน JSON เป็น [x, y, z]
                
                if position and len(position) >= 3:
                    # สร้าง Message แบบ Float32MultiArray
                    msg = Float32MultiArray()
                    
                    # ใส่ข้อมูลพิกัด x, y, z (และสามารถแถมเลข Action ID ถ้าต้องการ)
                    # เช่น: [x, y, z]
                    msg.data = [float(position[0]), float(position[1]), float(position[2])]
                    
                    self.pub_target.publish(msg)
                    self.get_logger().info(f"📍 Published Target: {msg.data}")
                    
                    self.provide_feedback(action, room)
                    self.current_state = STATE_WAITING_WAKEWORD
                    self.command_retry_count = 0
                else:
                    self.speak(f"ข้อมูลพิกัดของห้อง{self.rooms_dict[room]['response']}ไม่สมบูรณ์ครับ")
                    self.current_state = STATE_WAITING_WAKEWORD
            else:
                # ... Logic เดิมกรณีไม่เข้าใจคำสั่ง ...
                self.command_retry_count += 1
                if self.command_retry_count >= 3:
                    self.speak("เรียกผมใหม่นะครับ")
                    self.current_state = STATE_WAITING_WAKEWORD
                else:
                    self.speak("ไม่เข้าใจคำสั่งครับ ลองอีกครั้งนะ")

    def check_timeout(self):
        """ตรวจสอบว่ารอคำสั่งจากผู้ใช้นานเกินไปหรือไม่"""
        # --- [เพิ่มจุดนี้] ---
        # ถ้ากำลังประมวลผลเสียง หรือหุ่นยนต์กำลังพูดอยู่ ไม่ต้องเช็ค Timeout
        if self.is_processing:
            return

        if self.current_state == STATE_LISTENING_COMMAND:
            current_time = time.time()
            elapsed_time = current_time - self.last_interaction_time
            
            if elapsed_time > self.TIMEOUT_SECONDS:
                self.get_logger().info(f"⏰ Timeout reached ({elapsed_time:.1f}s)")
                self.current_state = STATE_WAITING_WAKEWORD
                # เมื่อเรียก speak() มันจะไปรีเซ็ตเวลาให้เองที่ตอนจบฟังก์ชัน
                self.speak("หมดเวลาครับ เรียกผมใหม่นะ")
                self.command_retry_count = 0

    def speak(self, text):
        if not text: 
            self.send_listen_signal(True)
            return
            
        # บอกโลกว่ากำลังพูด และปิดไมค์
        self.pub_speaking_status.publish(Bool(data=True))
        self.send_listen_signal(False)
        self.is_processing = True # ล็อคไว้ไม่ให้ check_timeout ทำงานแทรก
        
        try:
            self.get_logger().info(f"🔊 Robot speaking: {text}")
            tts = gTTS(text=text, lang='th')
            tts.save("feedback.mp3")
            playsound.playsound("feedback.mp3")
        finally:
            self.pub_speaking_status.publish(Bool(data=False))
            if os.path.exists("feedback.mp3"): os.remove("feedback.mp3")
            
            # --- [จุดสำคัญ] ---
            # 1. รีเซ็ตเวลาเพื่อเริ่มนับ 15 วิใหม่ "วินาทีที่พูดจบ"
            self.last_interaction_time = time.time() 
            # 2. ปลดล็อคสถานะประมวลผล
            self.is_processing = False
            # 3. สั่ง VAD เริ่มฟัง (ซึ่ง VAD ของคุณจะเริ่มนับ 8 วิของมันใหม่)
            self.send_listen_signal(True)
            self.get_logger().info("🔄 Timer & Mic Reset: Ready for next input.")
    
    def load_commands(self):
        try:
            pkg_share = get_package_share_directory('my_voice_control')
            json_path = os.path.join(pkg_share, 'commands.json')
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                self.wake_words = data.get("keyword_active", ["พี", "ปี", "ดี"])
                self.actions_list = data["actions"]
                self.rooms_dict = data["rooms"]
        except Exception as e:
            self.get_logger().warn(f"⚠️ Load fail: {e}")
            self.wake_words = ["พี", "ปี", "ดี"]; self.actions_list = []; self.rooms_dict = {}

    def find_action(self, text):
        for a in self.actions_list:
            if fuzz.partial_ratio(a, text) >= self.MATCH_THRESHOLD: return a
        return None

    def find_room(self, text):
        for r_key, r_val in self.rooms_dict.items():
            for cmd in r_val["commands"]:
                if fuzz.partial_ratio(str(cmd), text) >= self.MATCH_THRESHOLD: return r_key
        return None

    def provide_feedback(self, action, room):
        self.speak(f"รับทราบ กำลัง{action}ห้อง{self.rooms_dict[room].get('response', room)}ครับ")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()