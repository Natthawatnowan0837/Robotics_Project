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

STATE_WAITING_WAKEWORD = 0
STATE_LISTENING_COMMAND = 1

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('wave_to_text_node')
        self.is_processing = False
        # ROS Publishers & Subscribers
        self.sub_audio_ready = self.create_subscription(String, '/audio_ready', self.audio_ready_callback, 10)
        self.pub_whisper_done = self.create_publisher(Bool, '/whisper_done', 10)
        self.pub_target = self.create_publisher(String, '/robot_target', 10)
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
        self.MATCH_THRESHOLD = 85  # แนะนำให้ลดลงเหลือ 85 เพื่อให้สั่งงานง่ายขึ้น
        self.TIMEOUT_SECONDS = 8.0
        self.command_retry_count = 0

        # --- แก้ไขตรงนี้ ---
        # ลบ self.speak(...) ตัวเดิมออก
        # ใช้ Timer อย่างเดียวเพื่อให้ระบบเสถียรก่อนพูด
        self.greeting_timer = self.create_timer(2.0, self.say_greeting)
        self.timer = self.create_timer(1.0, self.check_timeout)

    def say_greeting(self):
        self.speak("สวัสดีครับ น้องพี พร้อมช่วยครับ")
        self.greeting_timer.cancel() # หยุด Timer เพื่อไม่ให้พูดซ้ำทุก 2 วินาที

    def audio_ready_callback(self, msg):
        if self.is_processing:
            return

        audio_file_path = msg.data
        if os.path.exists(audio_file_path):
            self.process_audio(audio_file_path)
        else:
            self.send_unlock_signal()

    def process_audio(self, audio_file):
        self.is_processing = True
        try:
            audio, sample_rate = sf.read(audio_file)
            input_features = self.processor(audio, sampling_rate=sample_rate, return_tensors="pt").input_features.to(self.device)

            with torch.no_grad():
                predicted_ids = self.model.generate(input_features, language='th', task='transcribe')
            raw_text = self.processor.batch_decode(predicted_ids, skip_special_tokens=True)[0].strip()
            if not raw_text:
                self.get_logger().info("... Silence or No speech detected ...")
                return

            print(f"\n🗣️ User said: {raw_text}")

            if self.current_state == STATE_WAITING_WAKEWORD:
                self.handle_wake_word(raw_text)
            else:
                self.handle_command(raw_text)

        except Exception as e:
            self.get_logger().error(f"❌ Error during processing: {e}")
        finally:
            if os.path.exists(audio_file):
                os.remove(audio_file)
            self.send_unlock_signal()
            self.is_processing = False

    def send_unlock_signal(self):
        msg = Bool()
        msg.data = True
        self.pub_whisper_done.publish(msg)

    def handle_wake_word(self, text):
        # ตรวจหา Wake word
        is_awake = any(kw in text for kw in self.wake_words)
        if not is_awake:
            for kw in self.wake_words:
                if len(kw) >= 3 and fuzz.partial_ratio(kw, text) >= self.MATCH_THRESHOLD:
                    is_awake = True
                    break

        if is_awake:
            self.current_state = STATE_LISTENING_COMMAND
            self.last_interaction_time = time.time()
            self.command_retry_count = 0 # Reset counter เมื่อเจอ Wake word ใหม่
            action = self.find_action(text)
            room = self.find_room(text)
            if action and room:
                print("⚡ เจอคำสั่งพ่วงมากับ Wake word เลย! ดำเนินการทันที...")
                self.handle_command(text)
            else:
                self.speak("ครับผม ว่าไงครับ")
        else:
            print(f"💤 รอคำเรียก {self.wake_words} (ได้ยินเป็น: {text})")

    def handle_command(self, text):
        action = self.find_action(text)
        room = self.find_room(text)

        if action and room:
            room_info = self.rooms_dict[room]
            position = room_info.get("position")
            if position and len(position) >= 3:
                target_data = {
                    "room_name": room,
                    "action": action,
                    "x": float(position[0]),
                    "y": float(position[1]),
                    "z": float(position[2])
                }
                for key, value in room_info.items():
                    if key not in ["commands", "position"]:
                        target_data[key] = value
                msg = String()
                msg.data = json.dumps(target_data, ensure_ascii=False)
                self.pub_target.publish(msg)
                self.provide_feedback(action, room)
                # สำเร็จแล้วกลับไปรอ Wake word
                self.current_state = STATE_WAITING_WAKEWORD
                self.command_retry_count = 0
            else:
                self.speak(f"ข้อมูลพิกัดของห้อง{self.rooms_dict[room]['response']}ไม่ถูกต้องครับ")
                self.current_state = STATE_WAITING_WAKEWORD
        else:
            # กรณีฟังคำสั่งไม่เข้าใจ
            self.command_retry_count += 1
            if self.command_retry_count >= 3:
                self.speak("ขอโทษครับ เรียกผมใหม่นะครับ")
                self.current_state = STATE_WAITING_WAKEWORD
                self.command_retry_count = 0
            else:
                self.speak("ขอโทษครับ ไม่เข้าใจคำสั่ง ลองอีกครั้งนะครับ")
                self.last_interaction_time = time.time() # ต่อเวลาให้พูดใหม่

    def check_timeout(self):
        if self.current_state == STATE_LISTENING_COMMAND:
            if time.time() - self.last_interaction_time > self.TIMEOUT_SECONDS:
                self.speak("หมดเวลาครับ เรียกผมใหม่นะ")
                self.current_state = STATE_WAITING_WAKEWORD
                self.command_retry_count = 0

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
                self.wake_words = data.get("keyword_active", ["พี", "ปี", "ดี"])
                self.actions_list = data["actions"]
                self.rooms_dict = data["rooms"]
        except Exception as e:
            self.get_logger().warn(f"⚠️ Could not load commands.json: {e}")
            self.wake_words = ["พี", "ปี", "ดี"]
            self.actions_list = []
            self.rooms_dict = {}

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