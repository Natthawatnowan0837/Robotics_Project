#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
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
        self.SILENCE_THRESHOLD = 0.6 # ปรับให้อ่อนลงหน่อยเผื่อเสียงเบา
        self.OUTPUT_FILENAME = "my_command.wav"
        
        # [NEW] ระบบ State Control ผ่าน Topic
        # 1. รับคำสั่ง: "ให้เริ่มฟังได้" (จาก Whisper หรือหุ่นยนต์ตอนพูดจบ)
        self.sub_listen = self.create_subscription(Bool, '/listen', self.listen_callback, 10)
        self.can_record = False # เริ่มต้นยังไม่ฟัง จนกว่าจะได้รับคำสั่งแรก

        # 2. ส่งสัญญาณ: "อัดเสร็จแล้วนะ" (บอก Whisper ให้เริ่มทำงาน)
        self.pub_state = self.create_publisher(Bool, '/voice_state', 10)

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
        self.get_logger().info("🎤 VAD Node Ready. Waiting for /listen = True")

    def listen_callback(self, msg):
        if msg.data:
            if not self.can_record:
                # --- [ADD] Flush Mic Buffer ---
                # อ่านข้อมูลค้างเก่าทิ้งไปให้หมดก่อนเริ่มฟังใหม่
                try:
                    while self.stream.get_read_available() > 0:
                        self.stream.read(self.stream.get_read_available(), exception_on_overflow=False)
                except:
                    pass
                
                self.get_logger().info("🔓 Mic Unlocked: Buffer Flushed & Listening...")
                self.reset_state() 
                self.can_record = True
        else:
            self.can_record = False
            self.get_logger().info("🔒 Mic Locked: Stopped Listening")

    def process_audio(self):
        if not self.can_record:
            return 

        try:
            data = self.stream.read(512, exception_on_overflow=False)
            audio_int16 = np.frombuffer(data, np.int16)
            audio_float32 = audio_int16.astype(np.float32) / 32768.0
            tensor = torch.from_numpy(audio_float32)
            speech_prob = self.model(tensor, 16000).item()

            if speech_prob > self.SILENCE_THRESHOLD:
                if not self.is_speaking:
                    self.get_logger().info("🔴 Started Recording...")
                    self.is_speaking = True
                    self.audio_buffer.extend(self.pre_buffer)
                self.silence_counter = 0 
                self.audio_buffer.append(data)
            else:
                if self.is_speaking:
                    self.audio_buffer.append(data)
                    self.silence_counter += 1
                    # ถ้าเงียบเกิน 0.8 วินาที (ประมาณ 25 chunks)
                    if self.silence_counter > 25:
                        self.save_and_signal()
                else:
                    self.pre_buffer.append(data)
        except Exception as e:
            self.get_logger().error(f"Mic Error: {e}")

    def save_and_signal(self):
        if len(self.audio_buffer) > 20:
            # 1. บันทึกไฟล์
            temp_filename = "recording_temp.wav"
            wf = wave.open(temp_filename, 'wb')
            wf.setnchannels(1)
            wf.setsampwidth(self.p.get_sample_size(pyaudio.paInt16))
            wf.setframerate(16000)
            wf.writeframes(b''.join(self.audio_buffer))
            wf.close()
            os.replace(temp_filename, self.OUTPUT_FILENAME)
            
            self.get_logger().info(f"💾 Saved. Signaling Whisper...")

            # 2. ปิดไมค์ตัวเอง (Lock) เพื่อไม่ให้บันทึกซ้อน
            self.can_record = False 

            # 3. ส่ง State ไปบอก Whisper
            state_msg = Bool()
            state_msg.data = True
            self.pub_state.publish(state_msg)
            
        self.reset_state()

    def reset_state(self):
        self.audio_buffer = []
        self.is_speaking = False
        self.silence_counter = 0
        self.pre_buffer.clear()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.stop_stream()
        node.stream.close()
        node.p.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()