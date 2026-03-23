#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
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

# --- [กำหนด States] ---
STATE_WAITING_WAKEWORD = 0
STATE_LISTENING_COMMAND = 1
STATE_WAITING_CONFIRMATION = 2

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('wave_to_text_node')
        self.is_processing = False
        
        # --- Publishers & Subscriptions ---
        self.sub_voice_state = self.create_subscription(Bool, '/voice_state', self.voice_state_callback, 10)
        self.pub_listen = self.create_publisher(Bool, '/listen', 10)
        self.pub_room_target = self.create_publisher(String, '/room_target', 10)
        self.pub_speaking_status = self.create_publisher(Bool, '/robot_is_speaking', 10)

        # --- Whisper Model Loading ---
        self.MODEL_NAME = "biodatlab/whisper-th-small-combined"
        self.get_logger().info(f"⏳ Loading Whisper Model...")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.processor = WhisperProcessor.from_pretrained(self.MODEL_NAME)
        self.model = WhisperForConditionalGeneration.from_pretrained(self.MODEL_NAME).to(self.device)
        self.get_logger().info(f"✅ Whisper Ready on {self.device}")

        # --- Configuration & State ---
        self.load_commands()
        self.current_state = STATE_WAITING_WAKEWORD
        self.pending_command = None 
        self.last_interaction_time = time.time()
        self.MATCH_THRESHOLD = 80 
        self.TIMEOUT_SECONDS = 15.0
        self.command_retry_count = 0
        self.AUDIO_FILE = "my_command.wav"

        # --- Timers ---
        self.greeting_timer = self.create_timer(2.0, self.say_greeting)
        self.timeout_timer = self.create_timer(1.0, self.check_timeout)

    def load_commands(self):
        try:
            pkg_share = get_package_share_directory('my_voice_control')
            json_path = os.path.join(pkg_share, 'commands.json')
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                self.wake_words = data.get("keyword_active", ["พี", "ปี", "ดี"])
                self.actions_list = data.get("actions", [])
                self.rooms_dict = data.get("rooms", {})
                self.yes_keywords = data.get("yes_keywords", ["ใช่", "ถูกต้อง", "ตกลง"])
                self.no_keywords = data.get("no_keywords", ["ไม่ใช่", "ไม่", "ยกเลิก"])
        except Exception as e:
            self.get_logger().error(f"⚠️ Load JSON fail: {e}")
            self.wake_words = ["พี"]; self.actions_list = []; self.rooms_dict = {}

    def say_greeting(self):
        self.greeting_timer.cancel() # รันครั้งเดียวแล้วปิด
        self.speak("สวัสดีครับ น้องพี พร้อมช่วยครับ")

    def voice_state_callback(self, msg):
        # รับสัญญาณว่าบันทึกเสียงเสร็จแล้ว
        if msg.data and not self.is_processing:
            if os.path.exists(self.AUDIO_FILE):
                self.process_audio(self.AUDIO_FILE)
            else:
                self.send_listen_signal(True)

    def process_audio(self, audio_file):
        self.is_processing = True
        self.send_listen_signal(False) # ปิดไมค์ขณะประมวลผล
        
        try:
            audio, sample_rate = sf.read(audio_file)
            input_features = self.processor(audio, sampling_rate=sample_rate, return_tensors="pt").input_features.to(self.device)
            
            with torch.no_grad():
                predicted_ids = self.model.generate(input_features, language='th', task='transcribe')
            
            raw_text = self.processor.batch_decode(predicted_ids, skip_special_tokens=True)[0].strip()
            
            if not raw_text:
                self.is_processing = False
                self.send_listen_signal(True)
                return

            self.get_logger().info(f"🗣️ User: {raw_text} | State: {self.current_state}")

            if self.current_state == STATE_WAITING_WAKEWORD:
                self.handle_wake_word(raw_text)
            elif self.current_state == STATE_LISTENING_COMMAND:
                self.handle_command(raw_text)
            elif self.current_state == STATE_WAITING_CONFIRMATION:
                self.handle_confirmation(raw_text)

        except Exception as e:
            self.get_logger().error(f"❌ Processing Error: {e}")
            self.is_processing = False
            self.send_listen_signal(True)
        finally:
            if os.path.exists(audio_file): 
                os.remove(audio_file)

    def handle_wake_word(self, text):
        is_wake = any(kw in text for kw in self.wake_words)
        if is_wake:
            self.last_interaction_time = time.time()
            action = self.find_keyword(text, self.actions_list)
            room = self.find_room(text)
            
            if action and room:
                self.ask_for_confirmation(action, room)
            else:
                self.current_state = STATE_LISTENING_COMMAND
                self.speak("ครับผม จะให้ทำอะไรดีครับ?")
        else:
            self.is_processing = False
            self.send_listen_signal(True)

    def handle_command(self, text):
        action = self.find_keyword(text, self.actions_list)
        room = self.find_room(text)

        if action and room:
            self.ask_for_confirmation(action, room)
        else:
            self.command_retry_count += 1
            if self.command_retry_count >= 2:
                self.speak("ขอโทษครับ ผมไม่เข้าใจคำสั่ง ไว้เรียกใหม่นะครับ")
                self.reset_to_start()
            else:
                self.speak("ไปที่ไหนนะครับ? ขออีกทีครับ")

    def ask_for_confirmation(self, action, room_key):
        self.pending_command = {"action": action, "room_key": room_key}
        room_response = self.rooms_dict[room_key].get("response", room_key)
        self.current_state = STATE_WAITING_CONFIRMATION
        self.last_interaction_time = time.time()
        self.speak(f"ต้องการให้{action}ที่ห้อง {room_response} ใช่ไหมครับ?")

    def handle_confirmation(self, text):
        clean_text = text.strip().replace(" ", "")
        is_no = any(kw in clean_text for kw in self.no_keywords)
        is_yes = False
        if not is_no:
            is_yes = any(kw in clean_text for kw in self.yes_keywords)

        if is_no:
            self.pending_command = None 
            self.current_state = STATE_LISTENING_COMMAND 
            self.speak("ขอโทษครับ งั้นไปที่ห้องไหนนะครับ?")
        elif is_yes:
            room_key = self.pending_command["room_key"]
            action = self.pending_command["action"]
            room_response = self.rooms_dict[room_key].get("response", "Target")
            
            msg = String()
            msg.data = room_response
            self.pub_room_target.publish(msg)
            
            self.speak(f"รับทราบครับ กำลัง{action}ที่ห้อง {room_response} ครับ")
            self.reset_to_start()
        else:
            self.speak("กรุณายืนยันอีกครั้งครับ ว่าใช่ หรือ ไม่ใช่?")

    def reset_to_start(self):
        self.current_state = STATE_WAITING_WAKEWORD
        self.pending_command = None
        self.command_retry_count = 0

    def find_keyword(self, text, keywords):
        for kw in keywords:
            if kw in text: return kw
            if fuzz.partial_ratio(kw, text) >= self.MATCH_THRESHOLD: return kw
        return None

    def find_room(self, text):
        clean_text = text.strip().replace(" ", "")
        for r_key, r_val in self.rooms_dict.items():
            for cmd in r_val["commands"]:
                if cmd.replace(" ", "") in clean_text:
                    return r_key
        
        best_match = None
        highest_score = 0
        for r_key, r_val in self.rooms_dict.items():
            for cmd in r_val["commands"]:
                score = fuzz.ratio(str(cmd), clean_text)
                if score > highest_score:
                    highest_score = score
                    best_match = r_key
        
        return best_match if highest_score >= 85 else None

    def send_listen_signal(self, status: bool):
        msg = Bool(); msg.data = status
        self.pub_listen.publish(msg)

    def check_timeout(self):
        if self.is_processing or self.current_state == STATE_WAITING_WAKEWORD:
            return
        if (time.time() - self.last_interaction_time) > self.TIMEOUT_SECONDS:
            self.speak("เงียบไปนาน ผมขอพักก่อนนะครับ")
            self.reset_to_start()

    def speak(self, text):
        if not text: 
            self.is_processing = False
            self.send_listen_signal(True)
            return

        self.is_processing = True
        self.send_listen_signal(False) # มั่นใจว่าปิดไมค์ขณะพูด
        self.pub_speaking_status.publish(Bool(data=True))
        
        try:
            self.get_logger().info(f"🔊 Robot: {text}")
            tts = gTTS(text=text, lang='th')
            tts.save("feedback.mp3")
            playsound.playsound("feedback.mp3")
        except Exception as e:
            self.get_logger().error(f"TTS Error: {e}")
        finally:
            if os.path.exists("feedback.mp3"): 
                try: os.remove("feedback.mp3")
                except: pass
            
            self.pub_speaking_status.publish(Bool(data=False))
            self.last_interaction_time = time.time()
            self.is_processing = False
            
            # หน่วงเวลาเล็กน้อยเพื่อเลี่ยงเสียงสะท้อนตอนท้าย
            time.sleep(0.5) 
            self.send_listen_signal(True)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandProcessor()
    
    # ใช้ MultiThreadedExecutor เพื่อให้จัดการ Timer และ Sub ได้พร้อมกัน
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Stopping Voice Command Node...')
    finally:
        # ล้าง Resource ทั้งหมดก่อนจบ
        node.greeting_timer.cancel()
        node.timeout_timer.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()