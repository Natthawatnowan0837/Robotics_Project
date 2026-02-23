#!/usr/bin/env python3
import rclpy
import torch
import json
import os
import soundfile as sf
import numpy as np
import sounddevice as sd
import playsound
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from transformers import WhisperProcessor, WhisperForConditionalGeneration
from gtts import gTTS

model_text = "biodatlab/whisper-th-small-combined"
processor = WhisperProcessor.from_pretrained(model_text)
whisper_model = WhisperForConditionalGeneration.from_pretrained(model_text)
device = "cuda" if torch.cuda.is_available() else "cpu"
whisper_model.to(device)


pkg_share = get_package_share_directory('my_voice_control')
json_path = os.path.join(pkg_share, 'commands.json')

with open(json_path, 'r', encoding='utf-8') as f:
    commands_json = json.load(f)

actions_list = commands_json["actions"]
rooms_dict = commands_json["rooms"]


class Voice_speech(Node):
    def __init__(self):
        super().__init__('voice_speech')

        # Publisher สำหรับส่ง action, room
        self.pub_cmd = self.create_publisher(String, '/voice_cmd', 10)

        # พูดเมื่อเริ่มต้นทำงาน
        self.speak("ต้องการให้ฉันขนของไปห้องไหนครับ")

        # Timer สำหรับฟังเสียงทุก 5 วินาที
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info("voice_speech node is running...")

    # ----------------------------
    #  ฟังก์ชันบันทึกเสียง
    # ----------------------------
    def record_audio(self, filename="user.wav", fs=16000):
        input("\nPress Enter for recording 5 second...")
        print("Recording...")
        recording = sd.rec(int(fs * 5), samplerate=fs, channels=1, dtype='float32')
        sd.wait()
        sf.write(filename, recording, fs)

    # ----------------------------
    #  ฟังก์ชันพูด (ใช้ gTTS)
    # ----------------------------
    def speak(self, text, lang="th"):
        if not text:
            return
        tts = gTTS(text=text, lang=lang)
        tts.save("voice_feedback.mp3")
        playsound.playsound("voice_feedback.mp3")

    # ----------------------------
    #  ฟังก์ชันฟังเสียงและแปลงเป็นข้อความ
    # ----------------------------
    def listen_command(self):
        self.record_audio()
        speech, sample_rate = sf.read("user.wav")
        speech = speech.astype(np.float32)
        inputs = processor(speech, sampling_rate=sample_rate, return_tensors="pt").to(device)

        with torch.no_grad():
            predicted_ids = whisper_model.generate(inputs.input_features)

        transcription = processor.batch_decode(predicted_ids, skip_special_tokens=True)[0]
        print(f"\nYou say: {transcription}")
        return transcription.lower()

    # ----------------------------
    #  ฟังก์ชันหา Action
    # ----------------------------
    def find_action(self, user_input):
        for action in actions_list:
            if action in user_input:
                print(f"action: {action}")
                return action
        print("no action found")
        return None

    # ----------------------------
    #  ฟังก์ชันหา Room
    # ----------------------------
    def find_room(self, user_input, phrase_dict):
        for key, phrases in phrase_dict.items():
            for phrase in phrases:
                if phrase in user_input:
                    print(f"room: {key} from '{phrase}'")
                    return key
        print("no room found")
        return None

    # ----------------------------
    #  Timer callback
    # ----------------------------
    def timer_callback(self):
        text = self.listen_command()
        action = self.find_action(text)
        room = self.find_room(text, {k: v["commands"] for k, v in rooms_dict.items()})

        msg = String()
        msg.data = json.dumps({
            "action": action if action else "",
            "room": room if room else "",
            "raw_text": text
        }, ensure_ascii=False)

        self.pub_cmd.publish(msg)
        self.get_logger().info(f"Published voice command: {msg.data}")

        # ----------------------------
        #  ตอบกลับด้วยเสียง
        # ----------------------------
        if action and room:
            # ดึงชื่อห้องจาก response
            room_response = rooms_dict[room]["response"]
            self.speak(f"ฉันกำลังไปห้อง{room_response}")
        elif action:
            self.speak(f"{action}")
        elif room:
            room_response = rooms_dict[room]["response"]
            self.speak(f"คำสั่งในห้อง{room_response}")
        else:
            self.speak("ไม่เข้าใจคำสั่ง")


def main(args=None):
    rclpy.init(args=args)
    node = Voice_speech()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
