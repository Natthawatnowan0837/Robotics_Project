from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),

            # ไฟล์ที่อยู่ที่ root ของ maps (เช่น rtabmap.db)
            (os.path.join('share', package_name, 'maps'), glob('maps/*.db')),

            # ไฟล์ข้างในโฟลเดอร์ floor ต่างๆ
            (os.path.join('share', package_name, 'maps/floor1'), glob('maps/floor1/*')),
            (os.path.join('share', package_name, 'maps/floor2'), glob('maps/floor2/*')),
            (os.path.join('share', package_name, 'maps/floor3'), glob('maps/floor3/*')),
            (os.path.join('share', package_name, 'maps/floor4'), glob('maps/floor4/*')),

            # ส่วนอื่นๆ (config, rviz, models) ทำเหมือนเดิม
            (os.path.join('share', package_name, 'config'), glob('config/*')),
            (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
            
            # models และ meshes (ตรวจให้ดีว่าไม่มีโฟลเดอร์ซ้อนใน meshes ถ้ามีต้องทำแบบเดียวกับ maps)
            (os.path.join('share', package_name, 'models/my_stair_robot'), glob('models/my_stair_robot/*.sdf')),
            (os.path.join('share', package_name, 'models/my_stair_robot'), glob('models/my_stair_robot/*.config')),
            (os.path.join('share', package_name, 'models/my_stair_robot'), glob('models/my_stair_robot/*.xacro')),
            (os.path.join('share', package_name, 'models/my_stair_robot/meshes'), glob('models/my_stair_robot/meshes/*')),
            
            # อย่าลืมเพิ่มโฟลเดอร์ launch ด้วยนะครับ!
            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noone',
    maintainer_email='Natthawatnowan@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_control = my_control.xbox_360:main',
            'pid_visualizer = my_control.analysis:main',
        ],
    },
)