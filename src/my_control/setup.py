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
            # ส่วนหนึ่งใน data_files ของ setup.py
            (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
            (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
            (os.path.join('share', package_name, 'config'), glob('config/*')),
            (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
            
            (os.path.join('share', package_name, 'my_stair_robot'), glob('my_stair_robot/*.xacro')),
            (os.path.join('share', package_name, 'my_stair_robot/meshes'), glob('my_stair_robot/meshes/*')),
            (os.path.join('share', package_name, 'my_stair_robot'), glob('my_stair_robot/*.config')),
    
    
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
            'controller_node = my_control.controller_node:main',
            'pid_parameters_node = my_control.pid_parameters:main',
            'client = my_control.client:main',
            'analysis_floor=my_control.analysis_floor:main',
            'obtacle_stop=my_control.obtacle_stop:main',
            'nav_goal=my_control.nav_goal:main'
        ],
    },
)