#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String  # เพิ่ม String
import torch
import numpy as np
import pyaudio
import wave
import os
from collections import deque

class VoiceCaptureNode(Node):
    def __init__(self):
        super().__init__('vad_to_wav')
        
        # --- Config ---
        self.SILENCE_THRESHOLD = 0.7   
        self.OUTPUT_FILENAME = "my_command.wav"
        
        # [NEW] ระบบ State Control
        # สร้าง Publisher เพื่อบอกโลกว่า "อัดเสร็จแล้วนะ"
        self.pub_state = self.create_publisher(String, '/voice_system_state', 10)
        
        # สร้าง Subscriber เพื่อรอฟังว่า "เริ่มอัดใหม่ได้เลย"
        self.sub_state = self.create_subscription(
            String, 
            '/voice_system_state', 
            self.state_callback, 
            10
        )
        
        # ตัวแปรควบคุม: เริ่มต้นให้ True (พร้อมอัดครั้งแรก)
        self.can_record = True 

        # [NEW] Subscriber รับสถานะว่าหุ่นยนต์พูดอยู่ไหม (เดิม)
        self.sub_speaking_status = self.create_subscription(
            Bool, '/robot_is_speaking', self.speaking_status_callback, 10
        )
        self.is_robot_speaking = False

        # --- Load VAD Model ---
        self.model, _ = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', trust_repo=True)
        
        # --- Setup Mic ---
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=512)

        # --- Variables ---
        self.audio_buffer = []       
        self.is_speaking = False     
        self.silence_counter = 0     
        self.pre_buffer = deque(maxlen=int(0.5 * 16000 / 512)) # 0.5s pre-buffer

        self.timer = self.create_timer(0.03, self.process_audio)
        self.get_logger().info("🎤 VAD Node Ready. Waiting for state: ready_to_record")

    def state_callback(self, msg):
        # ถ้าได้รับสถานะว่า Recording หรือ ready_to_record ให้ปลดล็อค
        if msg.data == "ready_to_record":
            if not self.can_record:
                self.get_logger().info("🔓 Ready to record next command.")
                self.can_record = True

    def speaking_status_callback(self, msg):
        self.is_robot_speaking = msg.data
        if self.is_robot_speaking:
            self.reset_state()

    def process_audio(self):
        try:
            data = self.stream.read(512, exception_on_overflow=False)

            # เงื่อนไขการหยุด: หุ่นยนต์พูดอยู่ OR ระบบยังไม่พร้อม (ยังประมวลผลอันเก่าไม่เสร็จ)
            if self.is_robot_speaking or not self.can_record:
                return 

            audio_int16 = np.frombuffer(data, np.int16)
            audio_float32 = audio_int16.astype(np.float32) / 32768.0
            tensor = torch.from_numpy(audio_float32)
            speech_prob = self.model(tensor, 16000).item()

            if speech_prob > self.SILENCE_THRESHOLD:
                if not self.is_speaking:
                    print("\n🔴 Started Recording...")
                    self.is_speaking = True
                    self.audio_buffer.extend(self.pre_buffer)
                self.silence_counter = 0 
                self.audio_buffer.append(data)
            else:
                if self.is_speaking:
                    self.audio_buffer.append(data)
                    self.silence_counter += 1
                    # ถ้าเงียบเกิน 0.75 วินาที (ประมาณ 23 chunks)
                    if self.silence_counter > 23:
                        self.check_and_save()
                        self.reset_state()
                else:
                    self.pre_buffer.append(data)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def check_and_save(self):
        if len(self.audio_buffer) < 15: # สั้นไปไม่บันทึก
            return
        
        self.save_wav_file()
        
        # [KEY] อัดเสร็จแล้ว ล็อคตัวเองไว้ก่อน และส่งสถานะบอก Node อื่น
        self.can_record = False
        state_msg = String()
        state_msg.data = "record_success"
        self.pub_state.publish(state_msg)
        print("🔒 Locked. Waiting for 'ready_to_record' state...")

    def save_wav_file(self):
        temp_filename = "recording_temp.wav"
        wf = wave.open(temp_filename, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(self.p.get_sample_size(pyaudio.paInt16))
        wf.setframerate(16000)
        wf.writeframes(b''.join(self.audio_buffer))
        wf.close()
        os.replace(temp_filename, self.OUTPUT_FILENAME)
        print(f"💾 Saved: '{self.OUTPUT_FILENAME}'")

    def reset_state(self):
        self.audio_buffer = []
        self.is_speaking = False
        self.silence_counter = 0
        self.pre_buffer.clear()

# ... main function เหมือนเดิม ...

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()