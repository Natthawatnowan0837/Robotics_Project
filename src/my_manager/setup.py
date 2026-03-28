from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_manager'

def get_data_files():
    # ไฟล์พื้นฐาน
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ]


    def add_recursive_files(source_dir):
        for root, dirs, files in os.walk(source_dir):
            if files:
                # สร้าง path ปลายทาง: share/my_control/models/... หรือ share/my_control/maps/...
                install_dir = os.path.join('share', package_name, root)
                file_list = [os.path.join(root, f) for f in files]
                data_files.append((install_dir, file_list))

    add_recursive_files('models')
    add_recursive_files('maps')
    
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noone',
    maintainer_email='Natthawatnowan@gmail.com',
    description='Package for my stair climbing robot control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_manager = my_manager.esp32_manager:main',
            'my_manager = my_manager.my_manager:main',
            'goal = my_manager.goal:main',
            'nav2 = my_manager.nav2:main',
            'check_floor = my_manager.check_floor:main',
            'check_localize = my_manager.check_localize:main',
            'check_position = my_manager.check_position:main',
            'open_map = my_manager.open_map:main',
            'rotation_control = my_manager.rotation_control:main',
            'controller = my_manager.controller:main'
        ],
    },
)