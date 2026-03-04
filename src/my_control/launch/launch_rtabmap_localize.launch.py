import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_control'
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
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'depth_module.profile': '640x480x15', # ลดความละเอียดและ FPS (มาตรฐานคือ 30fps ซึ่งหนักไป)
            'rgb_camera.profile': '640x480x15',    # ลด Resolution ของภาพสี (SLAM ไม่จำเป็นต้องใช้ Full HD)
            'pointcloud.enable': 'false',          # ปิดการสร้าง Pointcloud จากกล้อง (ให้ RTAB-Map ทำเองจะประหยัดกว่า)
            'clip_distance': '4.0',                # ตัดข้อมูลที่ไกลเกิน 4 เมตร (ลด Noise และการประมวลผล)

        }.items()
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
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

    # navigation_launch = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name), 'launch', 'navigation_launch.py'
    #             )]), launch_arguments={'use_sim_time': 'false'}.items()
    # )

    # 3. Static TF สำหรับเชื่อม IMU กับกล้อง
    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0.0', '0.0', '0.0', '-1.5708', '0.0', '0.0', 'camera_link', 'camera_imu_optical_frame']
    # )

    database_full_path = os.path.expanduser('~/.ros/rtabmap.db')

    # 4. RTAB-Map (ส่วนที่เพิ่มเข้ามา)
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'localization': 'true', 
            'database_path': database_full_path,         # ระบุที่เก็บแผนที่
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/depth/image_rect_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'base_link',
            'approx_sync': 'true',
            'wait_imu_to_init': 'true',
            'imu_topic': '/rtabmap/imu',
            'qos': '1',
            'rviz': 'true',
            'compressed': 'false',  # <--- เพิ่มบรรทัดนี้เพื่อปิดการใช้ compressed image
            'use_sim_time': 'false' # <--- ย้ำอีกครั้งว่าต้องเป็น false สำหรับหุ่นจริง
        }.items()
    )

    return LaunchDescription([
        realsense,
        rsp,
        node_joint_state_publisher,
        imu_filter,
        # static_tf,
        # แนะนำให้ใช้ TimerAction หน่วงเวลา RTAB-Map เล็กน้อยเพื่อให้กล้องและ IMU พร้อมก่อน
        TimerAction(period=3.0, actions=[rtabmap]),
    ])

    