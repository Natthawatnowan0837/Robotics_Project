import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. กล้อง RealSense
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
        )]),
        launch_arguments={
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'enable_sync': 'true',
            'align_depth.enable': 'true'
        }.items()
    )

    # 2. IMU Filter Madgwick
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu'
        }],
        remappings=[
            ('/imu/data_raw', '/camera/camera/imu'),
            ('/imu/data', '/rtabmap/imu')
        ]
    )

    # 3. Static TF สำหรับเชื่อม IMU กับกล้อง
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link', 'camera_imu_optical_frame']
    )

    # 4. RTAB-Map (ส่วนที่เพิ่มเข้ามา)
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/depth/image_rect_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'camera_link',
            'approx_sync': 'true',
            'wait_imu_to_init': 'true',
            'imu_topic': '/rtabmap/imu',
            'qos': '1',
            'rviz': 'true'
        }.items()
    )

    return LaunchDescription([
        realsense,
        imu_filter,
        static_tf,
        # แนะนำให้ใช้ TimerAction หน่วงเวลา RTAB-Map เล็กน้อยเพื่อให้กล้องและ IMU พร้อมก่อน
        TimerAction(period=3.0, actions=[rtabmap]),
    ])