from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. สำหรับไฟล์ config .yaml
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # 2. สำหรับไฟล์ .urdf และ .csv ในโฟลเดอร์ urdf
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.csv')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # 3. สำหรับไฟล์ทั้งหมดในโฟลเดอร์ meshes (เช่น .STL)
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
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
               'read=my_sim.read:main',
        ],
    },
)
