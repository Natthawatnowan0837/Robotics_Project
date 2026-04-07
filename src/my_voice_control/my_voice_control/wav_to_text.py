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
from transformers import WhisperProcessor, WhisperForConditionalGeneration
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
        
        # --- Publishers ---
        self.pub_listen = self.create_publisher(Bool, '/listen', 10)
        self.pub_room_target = self.create_publisher(String, '/room_target', 10)
        self.pub_tts = self.create_publisher(String, '/speaking_request', 10)

        # --- Subscriptions ---
        self.sub_voice_state = self.create_subscription(Bool, '/voice_state', self.voice_state_callback, 10)
        self.sub_tts_status = self.create_subscription(Bool, '/robot_is_speaking', self.tts_status_callback, 10)
        self.robot_busy = False 
        self.sub_robot_state = self.create_subscription(String, '/robot_current_state', self.robot_state_callback, 10)

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
        self.greeting_timer.cancel()
        self.send_tts_request("สวัสดีครับ น้องพี พร้อมช่วยครับ")

    def tts_status_callback(self, msg):
        if msg.data is False:
            self.get_logger().info("✅ TTS Finished speaking. Mic ON.")
            self.is_processing = False
            self.last_interaction_time = time.time()
            self.send_listen_signal(True)

    def voice_state_callback(self, msg):
        if msg.data and not self.is_processing:
            if os.path.exists(self.AUDIO_FILE):
                self.process_audio(self.AUDIO_FILE)
            else:
                self.send_listen_signal(True)

    def process_audio(self, audio_file):
        self.is_processing = True
        self.send_listen_signal(False) 
        
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

            self.get_logger().info(f"🗣️ User: {raw_text}")

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
            if os.path.exists(audio_file): os.remove(audio_file)

    def handle_wake_word(self, text):
        is_wake = any(kw in text for kw in self.wake_words)
        if is_wake:
            action = self.find_keyword(text, self.actions_list)
            room = self.find_room(text)
            if action and room:
                self.ask_for_confirmation(action, room)
            else:
                self.current_state = STATE_LISTENING_COMMAND
                self.send_tts_request("ครับผม จะให้ทำอะไรดีครับ?")
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
                self.send_tts_request("ขอโทษครับ ผมไม่เข้าใจคำสั่ง ไว้เรียกใหม่นะครับ")
                self.reset_to_start()
            else:
                self.send_tts_request("ไปที่ไหนนะครับ? ขออีกทีครับ")

    def ask_for_confirmation(self, action, room_key):
        self.pending_command = {"action": action, "room_key": room_key}
        room_response = self.rooms_dict[room_key].get("response", room_key)
        self.current_state = STATE_WAITING_CONFIRMATION
        self.send_tts_request(f"ต้องการให้{action}ที่ห้อง {room_response} ใช่ไหมครับ?")

    def handle_confirmation(self, text):
        clean_text = text.strip().replace(" ", "")
        is_no = any(kw in clean_text for kw in self.no_keywords)
        is_yes = any(kw in clean_text for kw in self.yes_keywords) if not is_no else False

        if is_no:
            self.get_logger().info("❌ User said NO. Going back to Listening Command.")
            self.current_state = STATE_LISTENING_COMMAND 
            self.send_tts_request("ขอโทษครับ งั้นไปที่ห้องไหนนะครับ?")
            
        elif is_yes:
            # เช็คสถานะหุ่นยนต์ก่อนเริ่มงานใหม่
            if self.robot_busy:
                self.get_logger().warn("⚠️ Robot is BUSY! Rejecting new mission.")
                self.send_tts_request("ขอโทษครับ ตอนนี้หุ่นยนต์กำลังทำงานอื่นอยู่ กรุณารอสักครู่ครับ")
                self.reset_to_start()
                return 

            # กรณีหุ่นยนต์ว่าง (IDLE): ส่ง Topic บอกเป้าหมาย
            room_key = self.pending_command["room_key"]
            action = self.pending_command["action"]
            room_response = self.rooms_dict[room_key].get("response", "Target")
            
            self.get_logger().info(f"✅ Confirmation received. Sending robot to {room_key}")

            # ส่งเฉพาะ Topic ไปให้ State Manager
            msg = String()
            msg.data = room_key 
            self.pub_room_target.publish(msg)
            
            # ตอบรับและ Reset
            self.send_tts_request(f"รับทราบครับ กำลัง{action}ที่ห้อง {room_response} ครับ")
            self.reset_to_start()
            
        else:
            self.get_logger().info("❓ Ambiguous answer. Asking again.")
            self.send_tts_request("ขอโทษครับ ใช่หรือไม่ครับ?")

    def reset_to_start(self):
        self.current_state = STATE_WAITING_WAKEWORD
        self.pending_command = None
        self.command_retry_count = 0

    def find_keyword(self, text, keywords):
        for kw in keywords:
            if kw in text or fuzz.partial_ratio(kw, text) >= self.MATCH_THRESHOLD: return kw
        return None

    def find_room(self, text):
        clean_text = text.strip().replace(" ", "")
        for r_key, r_val in self.rooms_dict.items():
            for cmd in r_val["commands"]:
                if cmd.replace(" ", "") in clean_text: return r_key
        return None

    def send_listen_signal(self, status: bool):
        msg = Bool(); msg.data = status
        self.pub_listen.publish(msg)

    def check_timeout(self):
        if self.is_processing or self.current_state == STATE_WAITING_WAKEWORD: return
        if (time.time() - self.last_interaction_time) > self.TIMEOUT_SECONDS:
            self.send_tts_request("เงียบไปนาน ผมขอพักก่อนนะครับ")
            self.reset_to_start()

    def send_tts_request(self, text):
        self.is_processing = True
        self.send_listen_signal(False) 
        msg = String()
        msg.data = text
        self.pub_tts.publish(msg)
        self.get_logger().info(f"🛰️ TTS Request: {text}")

    def robot_state_callback(self, msg):
        if msg.data == "IDLE":
            self.robot_busy = False
        else:
            self.robot_busy = True

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandProcessor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()