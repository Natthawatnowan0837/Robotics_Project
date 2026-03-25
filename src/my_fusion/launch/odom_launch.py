from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. รัน RealSense (ตามที่คุณตั้งค่าไว้)
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
        )]),
        launch_arguments={
            'depth_module.profile': '640,480,15',
            'rgb_module.profile': '640,480,15',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'enable_sync': 'true',
        }.items()
    )

    # 2. รัน RGBD Odometry เพื่อสร้าง Topic /visual_odom
    visual_odometry = Node(
        package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': False, # ให้ EKF เป็นคน Publish TF แทน เพื่อป้องกันการตีกัน
            'approx_sync': True,
            'wait_imu_to_init': True,
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('imu', '/camera/camera/imu/filtered'), # ใช้ IMU ในตัวกล้องช่วยคำนวณ Odom
            ('odom', '/visual_odom') # ส่งออกไปยัง Topic นี้เพื่อรอเข้า EKF
        ]
    )

    imu_filter = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[{
            'use_mag': False,       # RealSense ไม่มีเข็มทิศ
            'world_frame': 'enu',
            'publish_tf': False,
            'approx_sync_max_interval': 0.1,
            'wait_for_transform': 0.2,
        }],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),
            ('imu/data', '/camera/camera/imu/filtered') # ชื่อ Topic ใหม่ที่มี Orientation แล้ว
        ]
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('my_control'), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        realsense,
        visual_odometry,
        imu_filter,
        rsp,
        node_joint_state_publisher
    ])