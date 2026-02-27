#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool # [NEW]
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
        self.SILENCE_DURATION_LIMIT = 0.75
        self.MIN_SPEECH_DURATION = 0.5    
        self.PRE_RECORD_SECONDS = 0.5     
        self.OUTPUT_FILENAME = "my_command.wav"
        
        # [NEW] Subscriber รับสถานะว่าหุ่นยนต์พูดอยู่ไหม
        self.sub_speaking_status = self.create_subscription(
            Bool, 
            '/robot_is_speaking', 
            self.speaking_status_callback, 
            10
        )
        self.is_robot_speaking = False # สถานะเริ่มต้น

        # --- Load Model ---
        self.model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad',
                                      model='silero_vad',
                                      force_reload=False,
                                      trust_repo=True)
        self.get_logger().info("✅ VAD Model Loaded.")

        # --- Setup Mic ---
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.CHUNK = 512

        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=self.FORMAT,
                                  channels=self.CHANNELS,
                                  rate=self.RATE,
                                  input=True,
                                  frames_per_buffer=self.CHUNK)

        # --- Variables ---
        self.audio_buffer = []       
        self.is_speaking = False     
        self.silence_counter = 0     
        
        self.CHUNKS_PER_SECOND = self.RATE / self.CHUNK
        self.silence_limit_chunks = int(self.SILENCE_DURATION_LIMIT * self.CHUNKS_PER_SECOND)
        self.min_speech_chunks = int(self.MIN_SPEECH_DURATION * self.CHUNKS_PER_SECOND)
        
        maxlen = int(self.PRE_RECORD_SECONDS * self.CHUNKS_PER_SECOND)
        self.pre_buffer = deque(maxlen=maxlen)

        self.timer = self.create_timer(0.03, self.process_audio)
        self.get_logger().info(f"🎤 Ready! (Threshold: {self.SILENCE_THRESHOLD})")
        
    def speaking_status_callback(self, msg):
        # --- เพิ่มบรรทัดนี้ครับ ---
        self.get_logger().info(f"📥 Received '/robot_is_speaking': {msg.data}")
        # -----------------------

        self.is_robot_speaking = msg.data
        if self.is_robot_speaking:
            # ถ้าหุ่นเริ่มพูด ให้เคลียร์ State ทิ้งทันที (หยุดอัดกลางคัน)
            self.reset_state()
            print("\n🤖 Robot speaking... Mic paused.")
        else:
            print("👂 Robot finished. Listening again...")

    def process_audio(self):
        try:
            # ต้องอ่านค่าจากไมค์เสมอ เพื่อเคลียร์ Buffer ไม่ให้ Mic Overflow
            data = self.stream.read(self.CHUNK, exception_on_overflow=False)

            # [NEW] ถ้าหุ่นยนต์พูดอยู่ ให้โยนข้อมูลเสียงทิ้งไปเลย ไม่ต้องประมวลผล
            if self.is_robot_speaking:
                return 

            audio_int16 = np.frombuffer(data, np.int16)
            audio_float32 = audio_int16.astype(np.float32) / 32768.0
            
            tensor = torch.from_numpy(audio_float32)
            speech_prob = self.model(tensor, self.RATE).item()

            # --- Logic ---
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

                    if self.silence_counter > self.silence_limit_chunks:
                        self.check_and_save()
                        self.reset_state()
                else:
                    self.pre_buffer.append(data)
                    print(".", end='', flush=True)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def check_and_save(self):
        print("⏹️  Stop.")
        if len(self.audio_buffer) < self.min_speech_chunks:
            print(f"⚠️  Too short. Ignored.")
            return
        self.save_wav_file()

    def save_wav_file(self):
        filename = self.OUTPUT_FILENAME
        wf = wave.open(filename, 'wb')
        wf.setnchannels(self.CHANNELS)
        wf.setsampwidth(self.p.get_sample_size(self.FORMAT))
        wf.setframerate(self.RATE)
        wf.writeframes(b''.join(self.audio_buffer))
        wf.close()
        print(f"💾 Saved to '{filename}'")

    def reset_state(self):
        self.audio_buffer = []
        self.is_speaking = False
        self.silence_counter = 0
        self.pre_buffer.clear()

    def __del__(self):
        if hasattr(self, 'stream'):
            self.stream.stop_stream()
            self.stream.close()
        if hasattr(self, 'p'):
            self.p.terminate()

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