import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_voice_control'

    # 1. Node แปลงเสียง VAD เป็น WAV
    vad_to_wav = Node(
        package=package_name,
        executable='vad_to_wav',
        name='vad_to_wav_node',
        output='screen'
    )

    # 2. Node แปลง WAV เป็น Text (ตรวจสอบชื่อ executable ให้ตรงกับ setup.py)
    wav_to_text = Node(
        package=package_name,
        executable='wav_to_text', # แก้ไขจากเดิมที่เป็น vad_to_wav
        name='wav_to_text_node',
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'} # เพิ่มด้วย
    )

    # voice_speech = Node(
    #     package=package_name,
    #     executable='voice_speech', # แก้ไขจากเดิมที่เป็น vad_to_wav
    #     name='voice_speech_node',
    #     output='screen'
    # )
    # 3. Node ประมวลผลคำสั่งเสียง
    voice_processor = Node(
        package=package_name,
        executable='voice_processor', # มั่นใจว่าไม่มี space ข้างหน้าชื่อ
        name='voice_processor_node',
        output='screen'
    )

    return LaunchDescription([
        vad_to_wav,
        wav_to_text,
        # voice_speech,
        voice_processor
    ])