import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. กล้อง RealSense (หน้า 6)
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
        )]),
        launch_arguments={
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'enable_sync': 'true',
            'align_depth.enable': 'true' # แนะนำให้เปิดเพื่อให้ภาพตรงกัน [cite: 172, 176]
        }.items()
    )

    # 2. IMU Filter Madgwick (หน้า 7)
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[{
            'use_mag': False, # [cite: 190, 197]
            'publish_tf': False, # [cite: 198]
            'world_frame': 'enu' # [cite: 199]
        }],
        remappings=[
            ('/imu/data_raw', '/camera/camera/imu'), # [cite: 200]
            ('/imu/data', '/rtabmap/imu') # [cite: 201]
        ]
    )

    # 3. Static TF สำหรับเชื่อม IMU กับกล้อง (หน้า 7)
# แก้ไขส่วนที่ 3 ให้ชัวร์ว่าเชื่อมโยงถูกทิศทาง
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # ตรวจสอบว่า Parent คือ camera_link และ Child คือ frame ที่กล้องใช้จริง
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link', 'camera_imu_optical_frame'] #
    )

    return LaunchDescription([
        realsense,
        imu_filter,
        static_tf,
    ])