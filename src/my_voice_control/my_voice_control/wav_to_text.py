#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from ament_index_python.packages import get_package_share_directory
import torch
import os
import json
import time
import soundfile as sf
import playsound
from transformers import WhisperProcessor, WhisperForConditionalGeneration
from gtts import gTTS
from thefuzz import fuzz

# [NEW] Import Library สำหรับกรองเสียง
import numpy as np
import scipy.signal as signal
import noisereduce as nr

# สถานะของระบบ
STATE_WAITING_WAKEWORD = 0  
STATE_LISTENING_COMMAND = 1 

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_to_text')

        # --- Config ---
        self.AUDIO_FILE = "my_command.wav"
        self.MODEL_NAME = "biodatlab/whisper-th-small-combined"
        self.JSON_FILENAME = 'commands.json'
        self.TIMEOUT_SECONDS = 8.0 
        self.MATCH_THRESHOLD = 80 

        # Publisher ส่งคำสั่ง
        self.pub_cmd = self.create_publisher(String, '/voice_cmd', 10)
        
        # Publisher บอกสถานะการพูด
        self.pub_speaking_status = self.create_publisher(Bool, '/robot_is_speaking', 10)

        self.pub_feedback = self.create_publisher(String, '/robot_feedback', 10)

        # 1. Load Whisper
        self.get_logger().info(f"⏳ Loading Whisper Model: {self.MODEL_NAME} ...")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.processor = WhisperProcessor.from_pretrained(self.MODEL_NAME)
        self.model = WhisperForConditionalGeneration.from_pretrained(self.MODEL_NAME)
        self.model.to(self.device)
        self.get_logger().info(f"✅ Model Loaded on {self.device.upper()}")

        # 2. Load JSON
        self.load_commands()

        # 3. Variables
        self.current_state = STATE_WAITING_WAKEWORD
        self.last_interaction_time = 0

        self.timer = self.create_timer(0.1, self.main_loop)
        self.speak("ปูนพร้อมก่อ สุดหล่อพร้อมยาง")

    def load_commands(self):
        try:
            pkg_share = get_package_share_directory('my_voice_control')
            json_path = os.path.join(pkg_share, self.JSON_FILENAME)
            if not os.path.exists(json_path):
                json_path = os.path.join(os.getcwd(), self.JSON_FILENAME)

            with open(json_path, 'r', encoding='utf-8') as f:
                commands_json = json.load(f)

            self.keywords_active = commands_json.get("keyword_active", ["บอท"])
            self.actions_list = commands_json["actions"]
            self.rooms_dict = commands_json["rooms"]
            
        except Exception as e:
            self.get_logger().error(f"❌ Error loading JSON: {e}")
            self.keywords_active = ["บอท"]
            self.actions_list = []
            self.rooms_dict = {}

    def main_loop(self):
        if self.current_state == STATE_LISTENING_COMMAND:
            if time.time() - self.last_interaction_time > self.TIMEOUT_SECONDS:
                self.get_logger().info("⏰ Timeout! Resetting to Wake Word mode.")
                self.speak("หมดเวลาครับ เรียกผมใหม่นะ")
                self.current_state = STATE_WAITING_WAKEWORD
                if os.path.exists(self.AUDIO_FILE):
                    os.remove(self.AUDIO_FILE)
                return

        if os.path.exists(self.AUDIO_FILE):
            # รอสักนิดเพื่อให้มั่นใจว่าไฟล์ถูกเขียนเสร็จแล้วจริงๆ
            time.sleep(0.2) 
            self.process_audio()

    def apply_filters(self, audio_data, sample_rate):
        try:
            sos_main = signal.butter(10, [300, 3400], 'bandpass', fs=sample_rate, output='sos')
            filtered_audio = signal.sosfilt(sos_main, audio_data)

            # --- 2. Mid Frequency Boost (เน้นเสียงพูด) ---
            # เราจะสร้าง Filter อีกตัวที่กรองเอาเฉพาะย่าน 1000Hz - 2500Hz (ย่านที่ทำให้เสียงพูดชัด)
            # แล้วเอาไป "บวกเพิ่ม" ใส่เสียงเดิม ทำให้ย่านนี้ดังขึ้น
            sos_mid = signal.butter(2, [1000, 2500], 'bandpass', fs=sample_rate, output='sos')
            mid_frequencies = signal.sosfilt(sos_mid, filtered_audio)
            
            # Boost Factor: 0.5 = เพิ่มความดังย่านกลางขึ้น 50%
            # คุณสามารถปรับเลข 0.5 - 1.0 ได้ตามความชอบ
            boosted_audio = filtered_audio + (mid_frequencies * 0.8)

            # --- 3. Noise Reduction (ลดเสียงรบกวนพื้นหลัง) ---
            # ลดเสียงซ่าคงที่ (stationary noise)
            clean_audio = nr.reduce_noise(y=boosted_audio, sr=sample_rate, stationary=True, prop_decrease=0.6)

            # --- 4. Normalization (สำคัญมาก!) ---
            # เนื่องจากเรา Boost เสียงขึ้น ค่าอาจจะเกิน 1.0 หรือ -1.0 (เสียงแตก)
            # ต้องปรับระดับเสียงให้สูงสุดอยู่ที่ 0.9
            max_val = np.max(np.abs(clean_audio))
            if max_val > 0:
                clean_audio = clean_audio / max_val * 0.9

            return clean_audio

        except Exception as e:
            self.get_logger().warn(f"Filter Error: {e}. Using raw audio.")
            return audio_data

    def process_audio(self):
        try:
            self.get_logger().info(f"🎧 Processing... (State: {self.current_state})")
            
            # อ่านไฟล์เสียง
            audio, sample_rate = sf.read(self.AUDIO_FILE)

            # [NEW] เรียกใช้ Filter ก่อนส่งเข้า Whisper
            audio = self.apply_filters(audio, sample_rate)

            # Whisper Processing
            input_features = self.processor(
                audio, sampling_rate=sample_rate, return_tensors="pt"
            ).input_features.to(self.device)

            with torch.no_grad():
                predicted_ids = self.model.generate(input_features)
            
            raw_text = self.processor.batch_decode(predicted_ids, skip_special_tokens=True)[0]
            print("\n" + "="*50)
            print(f"🗣️  User said (Filtered): {raw_text}")
            print("="*50 + "\n")

            if self.current_state == STATE_WAITING_WAKEWORD:
                self.handle_wake_word(raw_text)
            elif self.current_state == STATE_LISTENING_COMMAND:
                self.handle_command(raw_text)

        except Exception as e:
            self.get_logger().error(f"Error processing: {e}")
        finally:
            if os.path.exists(self.AUDIO_FILE):
                os.remove(self.AUDIO_FILE)

    def handle_wake_word(self, text):
        is_called = False
        for kw in self.keywords_active:
            score = fuzz.partial_ratio(kw, text)
            if score >= self.MATCH_THRESHOLD:
                is_called = True
                break
        
        if is_called:
            self.speak("ครับผม ว่าไงครับ")
            self.current_state = STATE_LISTENING_COMMAND
            self.last_interaction_time = time.time()
        else:
            print("💤 Ignored")

    def handle_command(self, text):
        text_lower = text.lower()
        self.last_interaction_time = time.time() 

        action = self.find_action(text_lower)
        room = self.find_room(text_lower)

        if action or room:
            msg_payload = {
                "action": action if action else "N/A", 
                "room": room if room else "N/A",
                "raw_text": text
            }
            msg = String()
            msg.data = json.dumps(msg_payload, ensure_ascii=False)
            self.pub_cmd.publish(msg)
            self.get_logger().info(f"🚀 Published: {msg.data}")

            self.provide_feedback(action, room)
            self.current_state = STATE_WAITING_WAKEWORD
        else:
            self.speak("ขอโทษครับ ไม่เข้าใจคำสั่ง ลองพูดใหม่อีกทีครับ")

    def find_action(self, user_input):
        best_action = None
        highest_score = 0
        for action in self.actions_list:
            score = fuzz.partial_ratio(action, user_input)
            if score >= self.MATCH_THRESHOLD and score > highest_score:
                highest_score = score
                best_action = action
        return best_action

    def find_room(self, user_input):
        best_room_key = None
        highest_score = 0
        for room_key, room_data in self.rooms_dict.items():
            for phrase in room_data["commands"]:
                score = fuzz.partial_ratio(str(phrase), user_input)
                if score >= self.MATCH_THRESHOLD and score > highest_score:
                    highest_score = score
                    best_room_key = room_key
        return best_room_key

    def provide_feedback(self, action, room):
        if action and room:
            room_resp = self.rooms_dict[room]["response"]
            self.speak(f"รับทราบ กำลัง{action}ห้อง{room_resp}ครับ")
        elif action:
            self.speak(f"รับทราบ จะทำการ{action}ครับ")
        elif room:
            room_resp = self.rooms_dict[room]["response"]
            self.speak(f"เกี่ยวกับห้อง{room_resp}นะครับ")

    def speak(self, text, lang="th"):
            if not text: return
            try:
                # สร้าง message
                msg = String(data=text)
                
                # 1. ส่ง Topic ออกไป
                self.pub_feedback.publish(msg)
                
                # [NEW] เพิ่มบรรทัดนี้ เพื่อโชว์ Log ว่าส่งแล้ว
                self.get_logger().info(f"📡 Published /robot_feedback: '{text}'")

                # 2. ส่งสถานะว่ากำลังพูด
                self.pub_speaking_status.publish(Bool(data=True)) 
                
                # ... (ส่วนสร้างไฟล์เสียงเหมือนเดิม) ...
                filename = "voice_feedback.mp3"
                if os.path.exists(filename): os.remove(filename)
                
                tts = gTTS(text=text, lang=lang)
                tts.save(filename)
                playsound.playsound(filename)
                
            except Exception as e:
                self.get_logger().error(f"TTS Error: {e}")
            
            finally:
                self.pub_speaking_status.publish(Bool(data=False))

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()