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

# สถานะภายใน (Logic ของการคุย)
STATE_WAITING_WAKEWORD = 0  
STATE_LISTENING_COMMAND = 1 

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('wave_to_text_node')
        
        # --- [NEW] Flags & State Control ---
        self.is_processing = False
        self.AUDIO_FILE = "my_command.wav"
        
        # Publisher/Subscriber สำหรับจัดการลำดับงาน (Handshaking)
        self.pub_system_state = self.create_publisher(String, '/voice_system_state', 10)
        self.sub_system_state = self.create_subscription(
            String, '/voice_system_state', self.system_state_callback, 10
        )

        # Publisher อื่นๆ
        self.pub_cmd = self.create_publisher(String, '/voice_cmd', 10)
        self.pub_speaking_status = self.create_publisher(Bool, '/robot_is_speaking', 10)
        self.pub_feedback = self.create_publisher(String, '/robot_feedback', 10)

        # --- Whisper Config ---
        self.MODEL_NAME = "biodatlab/whisper-th-small-combined"
        self.get_logger().info(f"⏳ Loading Whisper Model...")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.processor = WhisperProcessor.from_pretrained(self.MODEL_NAME)
        self.model = WhisperForConditionalGeneration.from_pretrained(self.MODEL_NAME).to(self.device)
        self.get_logger().info(f"✅ Whisper Ready on {self.device}")

        # --- Load JSON & Variables ---
        self.load_commands()
        self.current_state = STATE_WAITING_WAKEWORD
        self.last_interaction_time = time.time()
        self.MATCH_THRESHOLD = 80
        self.TIMEOUT_SECONDS = 8.0

        # Timer สำหรับเช็ค Timeout เท่านั้น (ไม่เช็คไฟล์แล้ว)
        self.timer = self.create_timer(1.0, self.check_timeout)

    def system_state_callback(self, msg):
        # [KEY] ถ้าได้รับแจ้งว่าอัดเสียงเสร็จแล้ว และตอนนี้ไม่ได้กำลังยุ่งอยู่
        if msg.data == "record_success" and not self.is_processing:
            if os.path.exists(self.AUDIO_FILE):
                self.process_audio()
            else:
                self.get_logger().warn("⚠️ State says success but file not found!")

    def process_audio(self):
        self.is_processing = True
        try:
            self.get_logger().info(f"🎧 Processing Audio (Internal State: {self.current_state})")
            
            # อ่านไฟล์และแปลงเสียง
            audio, sample_rate = sf.read(self.AUDIO_FILE)
            input_features = self.processor(audio, sampling_rate=sample_rate, return_tensors="pt").input_features.to(self.device)

            with torch.no_grad():
                predicted_ids = self.model.generate(input_features, language='th', task='transcribe')
            
            raw_text = self.processor.batch_decode(predicted_ids, skip_special_tokens=True)[0].strip()
            
            if not raw_text:
                self.get_logger().info("🔇 No text detected.")
                self.send_system_ready() # ปลดล็อคให้ไมค์ทำงานต่อ
                return

            print(f"\n🗣️ User said: {raw_text}")

            # แยก Logic ตามสถานะ (Wake Word หรือ Command)
            if self.current_state == STATE_WAITING_WAKEWORD:
                self.handle_wake_word(raw_text)
            else:
                self.handle_command(raw_text)

        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")
            self.send_system_ready()
        finally:
            if os.path.exists(self.AUDIO_FILE):
                os.remove(self.AUDIO_FILE)
            self.is_processing = False

    def handle_wake_word(self, text):
        if any(kw in text for kw in self.keywords_active) or \
           any(fuzz.partial_ratio(kw, text) >= self.MATCH_THRESHOLD for kw in self.keywords_active):
            
            self.speak("ครับผม ว่าไงครับ")
            self.current_state = STATE_LISTENING_COMMAND
            self.last_interaction_time = time.time()
            # ปลดล็อคให้ระบบเริ่มอัดคำสั่งต่อ
            self.send_system_ready()
        else:
            print("💤 Wake word not found.")
            self.send_system_ready()

    def handle_command(self, text):
        action = self.find_action(text)
        room = self.find_room(text)

        if action or room:
            # ส่งคำสั่งไปยังหุ่นยนต์
            payload = {"action": action or "N/A", "room": room or "N/A", "raw_text": text}
            self.pub_cmd.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            
            self.provide_feedback(action, room)
            self.current_state = STATE_WAITING_WAKEWORD
            
            # [NEW] ส่งสถานะว่าประมวลผลคำสั่งเสร็จสิ้นแล้ว
            self.send_state("command_success")
            time.sleep(0.5)
            self.send_system_ready()
        else:
            self.speak("ขอโทษครับ ไม่เข้าใจคำสั่ง")
            self.send_system_ready()

    def send_state(self, state_str):
        self.pub_system_state.publish(String(data=state_str))
        self.get_logger().info(f"📡 System State -> {state_str}")

    def send_system_ready(self):
        # ปลดล็อคให้ฝั่ง vad_to_wav เริ่มอัดใหม่ได้
        self.send_state("ready_to_record")

    def check_timeout(self):
        if self.current_state == STATE_LISTENING_COMMAND:
            if time.time() - self.last_interaction_time > self.TIMEOUT_SECONDS:
                self.speak("หมดเวลาครับ เรียกผมใหม่นะ")
                self.current_state = STATE_WAITING_WAKEWORD
                self.send_system_ready()

    # --- [ Helper Functions: Load JSON, Find Action, Speak ] ---
    # (ส่วนนี้ใช้ Code เดิมของคุณได้เลยครับ)
    def speak(self, text):
        if not text: return
        self.pub_speaking_status.publish(Bool(data=True))
        try:
            tts = gTTS(text=text, lang='th')
            tts.save("feedback.mp3")
            playsound.playsound("feedback.mp3")
        finally:
            self.pub_speaking_status.publish(Bool(data=False))
            if os.path.exists("feedback.mp3"): os.remove("feedback.mp3")

    def load_commands(self):
        try:
            pkg_share = get_package_share_directory('my_voice_control')
            json_path = os.path.join(pkg_share, 'commands.json')
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                self.keywords_active = data.get("keyword_active", ["บอท"])
                self.actions_list = data["actions"]
                self.rooms_dict = data["rooms"]
        except:
            self.keywords_active = ["บอท"]; self.actions_list = []; self.rooms_dict = {}

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
        if action and room:
            self.speak(f"รับทราบ กำลัง{action}ห้อง{self.rooms_dict[room]['response']}ครับ")
        elif action: self.speak(f"รับทราบ จะทำการ{action}ครับ")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()