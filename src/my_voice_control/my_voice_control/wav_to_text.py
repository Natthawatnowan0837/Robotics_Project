#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from std_msgs.msg import String, Bool
import torch
import os
import json
import time
import soundfile as sf
from transformers import WhisperProcessor, WhisperForConditionalGeneration
from thefuzz import fuzz
from ament_index_python.packages import get_package_share_directory

# --- [กำหนด Logic Steps] ---
STEP_WAITING_WAKEWORD = "WAIT_WAKE"
STEP_LISTENING_COMMAND = "WAIT_CMD"
STEP_WAITING_CONFIRMATION = "WAIT_CONFIRM"

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')
        self.is_processing = False
        
        # --- Publishers ---
        self.pub_listen = self.create_publisher(Bool, '/listen', 10)
        self.pub_room_target = self.create_publisher(String, '/room_target', 10)
        self.pub_tts = self.create_publisher(String, '/speaking_request', 10)
        self.pub_voice_state = self.create_publisher(String, '/voice_state', 10)

        # --- Subscriptions ---
        self.sub_voice_state = self.create_subscription(String, '/voice_state', self.voice_state_callback, 10)
        # ตัดการ Subscribe /process ออกไปเลย ไม่ต้องเช็คสถานะหุ่นยนต์แล้ว

        # --- Whisper Model Loading ---
        self.MODEL_NAME = "biodatlab/whisper-th-small-combined"
        self.get_logger().info(f"⏳ Loading Whisper Model...")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.processor = WhisperProcessor.from_pretrained(self.MODEL_NAME)
        self.model = WhisperForConditionalGeneration.from_pretrained(self.MODEL_NAME).to(self.device)
        self.get_logger().info(f"✅ Whisper Ready on {self.device} (Always Listening Mode)")

        # --- Configuration & State ---
        self.load_commands()
        self.current_logic_step = STEP_WAITING_WAKEWORD
        self.pending_command = None 
        self.last_interaction_time = time.time()
        self.MATCH_THRESHOLD = 80 
        self.AUDIO_FILE = "my_command.wav"

        self.create_timer(1.0, self.check_timeout)

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
                self.stop_keywords = ["หยุด", "จอด", "เลิกทำงาน", "stop"]
        except Exception as e:
            self.get_logger().error(f"⚠️ Load JSON fail: {e}")

    # --- [ Callback Functions ] ---

    def voice_state_callback(self, msg):
        if msg.data == "process" and not self.is_processing:
            if os.path.exists(self.AUDIO_FILE):
                self.process_audio(self.AUDIO_FILE)

    def process_audio(self, audio_file):
        self.is_processing = True
        try:
            audio, sample_rate = sf.read(audio_file)
            input_features = self.processor(audio, sampling_rate=sample_rate, return_tensors="pt").input_features.to(self.device)
            with torch.no_grad():
                predicted_ids = self.model.generate(input_features, language='th', task='transcribe')
            raw_text = self.processor.batch_decode(predicted_ids, skip_special_tokens=True)[0].strip()
            
            if not raw_text:
                self.reset_to_listen()
                return

            self.get_logger().info(f"🗣️ Transcribed: {raw_text}")

            # [LOGIC: EMERGENCY STOP] 
            is_stop_cmd = any(kw in raw_text or fuzz.partial_ratio(kw, raw_text) >= 90 for kw in self.stop_keywords)
            if is_stop_cmd:
                self.get_logger().warn("🚨 STOP COMMAND RECEIVED!")
                self.send_robot_response("รับทราบครับ หยุดทำงานทันที")
                stop_msg = String(); stop_msg.data = "IDLE" 
                self.pub_room_target.publish(stop_msg)
                self.current_logic_step = STEP_WAITING_WAKEWORD
                return

            # ตัด BUSY LOGIC ออกทั้งหมด รับคำสั่งได้ทันที
            self.last_interaction_time = time.time()
            if self.current_logic_step == STEP_WAITING_WAKEWORD:
                self.handle_wake_word(raw_text)
            elif self.current_logic_step == STEP_LISTENING_COMMAND:
                self.handle_command(raw_text)
            elif self.current_logic_step == STEP_WAITING_CONFIRMATION:
                self.handle_confirmation(raw_text)

        except Exception as e:
            self.get_logger().error(f"❌ Whisper Error: {e}")
            self.reset_to_listen()
        finally:
            if os.path.exists(audio_file): os.remove(audio_file)
            self.is_processing = False

    # --- [ Logic Handling Functions ] ---

    def handle_wake_word(self, text):
        is_wake = any(kw in text for kw in self.wake_words)
        if is_wake:
            action = self.find_keyword(text, self.actions_list)
            room = self.find_room(text)
            if action and room:
                self.ask_for_confirmation(action, room)
            else:
                self.current_logic_step = STEP_LISTENING_COMMAND
                self.send_robot_response("ครับผม จะให้ทำอะไรดีครับ?")
        else:
            self.reset_to_listen()

    def handle_command(self, text):
        action = self.find_keyword(text, self.actions_list)
        room = self.find_room(text)
        if action and room:
            self.ask_for_confirmation(action, room)
        else:
            self.send_robot_response("ไปที่ห้องไหนนะครับ?")

    def ask_for_confirmation(self, action, room_key):
        self.pending_command = {"action": action, "room_key": room_key}
        room_response = self.rooms_dict[room_key].get("response", room_key)
        self.current_logic_step = STEP_WAITING_CONFIRMATION
        self.send_robot_response(f"ต้องการให้{action}ที่ห้อง {room_response} ใช่ไหมครับ?")

    def handle_confirmation(self, text):
        clean_text = text.strip().replace(" ", "")
        is_no = any(kw in clean_text for kw in self.no_keywords)
        is_yes = any(kw in clean_text for kw in self.yes_keywords) if not is_no else False

        if is_no:
            self.current_logic_step = STEP_LISTENING_COMMAND 
            self.send_robot_response("งั้นไปที่ห้องไหนนะครับ?")
        elif is_yes:
            room_key = self.pending_command["room_key"]
            action = self.pending_command["action"]
            room_response = self.rooms_dict[room_key].get("response", "Target")
            self.send_robot_response(f"รับทราบครับ กำลัง{action}ที่ห้อง {room_response} ครับ")
            
            msg = String(); msg.data = room_key 
            self.pub_room_target.publish(msg)
            
            self.current_logic_step = STEP_WAITING_WAKEWORD
            self.pending_command = None
        else:
            self.send_robot_response("ใช่ หรือ ไม่ใช่ ครับ?")

    # --- [ Helper Functions ] ---

    def send_robot_response(self, text):
        msg = String(); msg.data = text
        self.pub_tts.publish(msg)
        state_msg = String(); state_msg.data = "speak"
        self.pub_voice_state.publish(state_msg)
        self.get_logger().info(f"📢 Robot Response: {text}")

    def reset_to_listen(self):
        self.is_processing = False
        msg = Bool(); msg.data = True
        self.pub_listen.publish(msg)

    def check_timeout(self):
        if self.is_processing or self.current_logic_step == STEP_WAITING_WAKEWORD: return
        if (time.time() - self.last_interaction_time) > 15.0:
            self.send_robot_response("เงียบไปนาน ผมขอพักก่อนนะครับ")
            self.current_logic_step = STEP_WAITING_WAKEWORD

    def find_keyword(self, text, keywords):
        for kw in keywords:
            if kw in text or fuzz.partial_ratio(kw, text) >= self.MATCH_THRESHOLD:
                return kw
        return None

    def find_room(self, text):
        clean_text = text.strip().replace(" ", "")
        for r_key, r_val in self.rooms_dict.items():
            for cmd in r_val["commands"]:
                if cmd.replace(" ", "") in clean_text:
                    return r_key
        return None

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandProcessor()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('🛑 Voice Processor stopping...')
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()