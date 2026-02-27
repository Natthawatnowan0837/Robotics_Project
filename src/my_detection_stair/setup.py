from setuptools import find_packages, setup
from setuptools import setup
import os
from glob import glob
package_name = 'my_detection_stair'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 1. ติดตั้งไฟล์ Launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 2. ติดตั้งไฟล์ RViz config
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # 3. ติดตั้งไฟล์โมเดล .pth (จากรูปอยู่ด้านนอกสุด)
        (os.path.join('share', package_name), glob('*.pth')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jo',
    maintainer_email='Natthawatnowan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_to_pointclound = detection_stair.depth_to_pointclound:main',
            'train_model = detection_stair.train_model:main',
            'test_model = detection_stair.test_model:main',
        ],
    },
)
