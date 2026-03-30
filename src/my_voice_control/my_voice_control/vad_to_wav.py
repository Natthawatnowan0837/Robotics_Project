#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String # เพิ่ม String
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
        self.SILENCE_THRESHOLD = 0.6 
        self.OUTPUT_FILENAME = "my_command.wav"
        
        # --- [SYSTEM CONTROL] ---
        # รับ Bool: True เพื่อปลดล็อกให้เริ่มฟังใหม่
        self.sub_listen = self.create_subscription(Bool, '/listen', self.listen_callback, 10)
        
        # ส่ง String: "process" เมื่ออัดเสียงเสร็จและบันทึกไฟล์เรียบร้อย
        self.pub_state = self.create_publisher(String, '/voice_state', 10)
        
        # เริ่มต้น: ให้เปิดฟังทันทีในครั้งแรกที่รัน (First Start)
        self.can_record = True 
        self.get_logger().info("🎤 VAD Node Ready. [FIRST RUN]: Automatically Listening...")

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

    def listen_callback(self, msg):
        """ รับสัญญาณปลดล็อกไมค์ """
        if msg.data: # ถ้าได้รับ True
            if not self.can_record:
                # เคลียร์ Buffer เก่าที่อาจค้างอยู่ใน Mic Stream
                try:
                    while self.stream.get_read_available() > 0:
                        self.stream.read(self.stream.get_read_available(), exception_on_overflow=False)
                except:
                    pass
                
                self.get_logger().info("🔓 Mic Unlocked: Waiting for voice...")
                self.reset_state() 
                self.can_record = True
        else:
            # ถ้าได้รับ False ให้หยุดฟังทันที
            self.can_record = False
            self.get_logger().info("🔒 Mic Locked manually")

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
                    self.get_logger().info("🔴 Detected Voice: Recording...")
                    self.is_speaking = True
                    self.audio_buffer.extend(self.pre_buffer)
                self.silence_counter = 0 
                self.audio_buffer.append(data)
            else:
                if self.is_speaking:
                    self.audio_buffer.append(data)
                    self.silence_counter += 1
                    # เงียบเกิน 0.8 วินาที
                    if self.silence_counter > 25:
                        self.save_and_signal()
                else:
                    self.pre_buffer.append(data)
        except Exception as e:
            self.get_logger().error(f"Mic Error: {e}")

    def save_and_signal(self):
        """ บันทึกไฟล์, Lock ไมค์ และส่งสัญญาณไป Process """
        if len(self.audio_buffer) > 20:
            temp_filename = "recording_temp.wav"
            wf = wave.open(temp_filename, 'wb')
            wf.setnchannels(1)
            wf.setsampwidth(self.p.get_sample_size(pyaudio.paInt16))
            wf.setframerate(16000)
            wf.writeframes(b''.join(self.audio_buffer))
            wf.close()
            os.replace(temp_filename, self.OUTPUT_FILENAME)
            
            self.get_logger().info(f"💾 Saved {self.OUTPUT_FILENAME}. Locking Mic...")

            # --- LOCK ไมค์ทันทีเพื่อรอการประมวลผลจาก Node อื่น ---
            self.can_record = False 

            # --- ส่ง Topic เป็น String "process" ---
            state_msg = String()
            state_msg.data = "process"
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