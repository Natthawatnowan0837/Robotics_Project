from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_voice_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # เพิ่มไฟล์ Launch เข้าไปในระบบ
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # ติดตั้งไฟล์ commands.json
        ('share/' + package_name, ['commands.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noone',
    maintainer_email='Natthawatnowan@gmail.com',
    description='Voice control with VAD for robot navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vad_to_wav = my_voice_control.vad_to_wav:main',
            'wav_to_text = my_voice_control.wav_to_text:main',
            'voice_speaking = my_voice_control.voice_speaking:main',
        ],
    },
)