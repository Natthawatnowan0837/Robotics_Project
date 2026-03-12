import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression # เพิ่ม LaunchConfiguration เข้าไป
def generate_launch_description():
    package_name = 'my_control'

    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'localized.rviz'
    )
    
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
        )]),
        launch_arguments={
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '1',
            'enable_sync': 'true',
            'depth_module.depth_visualization': 'true', # แถม: สำหรับดูภาพใน RViz ง่ายขึ้น
        }.items()
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('my_sim'), 'launch', 'rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items() # เปลี่ยนเป็น false
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
     
    # เพิ่ม .db ต่อท้ายชื่อไฟล์ที่ต้องการบันทึก
    database_full_path = os.path.expanduser('/home/noone/Robotics_Project/src/my_control/map/floor1/go/go.db')

    # 4. RTAB-Map (ปรับปรุงให้เหมือน Terminal)
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'database_path': database_full_path,
            'rtabmap_args': '--delete_db_on_start', 
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'base_link',
            'approx_sync': 'true',         # ตั้งเป็น true เพื่อความยืดหยุ่นในการรับข้อมูล
            'wait_imu_to_init': 'true',    # รอ IMU ให้พร้อมก่อนเริ่ม SLAM
            'imu_topic': '/rtabmap/imu',
            'qos': '1',
            'rviz': 'true',                # เปิด RViz อัตโนมัติ
            'rviz_cfg': rviz_config_path   # ใช้ไฟล์ config ที่คุณตั้งไว้
        }.items()
    )

    return LaunchDescription([
        realsense,
        rsp,
        node_joint_state_publisher,
        imu_filter,
        # ใช้ TimerAction เพื่อรอให้ Realsense และ IMU Filter ปล่อยข้อมูลออกมาก่อน 3 วินาที
        TimerAction(period=3.0, actions=[rtabmap]),
    ])