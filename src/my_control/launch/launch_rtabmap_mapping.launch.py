import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression # เพิ่ม LaunchConfiguration เข้าไป
def generate_launch_description():
    package_name = 'my_control'
    # 1. กล้อง RealSense
# ... ภายในฟังก์ชัน generate_launch_description ...

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
            # หากต้องการจำกัดระยะ 4 เมตรตามคอมเมนต์ของคุณ:
            # 'clip_distance': '4.0' 
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

    controller =  Node(
            package='my_control', # ชื่อ package
            executable='xbox_controller_node', # ชื่อที่ตั้งไว้ใน setup.py หรือชื่อไฟล์
            name='xbox_controller_node', # ชื่อ node ตอนรัน (optional)
            parameters=[{'mode': 'map'}], # ส่ง parameter (optional)
            output='screen' # ให้แสดง log บนหน้าจอ terminal
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

    database_full_path = os.path.expanduser('/home/noone/Robotics_Project/src/my_control/map/floor1/back/rtabmap.db')

    # 4. RTAB-Map (ส่วนที่เพิ่มเข้ามา)
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start', # นี่คือตัวแทนของ '-d' เพื่อลบแผนที่เก่าและเริ่มใหม่
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'base_link',
            'approx_sync': 'false', # ใช้ false เพราะเราเปิด sync ที่ตัวกล้องแล้ว
            'imu_topic': '/rtabmap/imu',
            'wait_imu_to_init': 'true',
            'qos': '1', # 1 = RMW_QOS_POLICY_RELIABILITY_RELIABLE
            'rviz': 'true' # เปิด RViz อัตโนมัติหรือไม่ (ตั้งเป็น false ได้ถ้าจะรันแยก)
        }.items()
    )

    return LaunchDescription([
        realsense,
        rsp,
        node_joint_state_publisher,
        imu_filter,
        controller,
        # navigation_launch,
        # static_tf,
        # แนะนำให้ใช้ TimerAction หน่วงเวลา RTAB-Map เล็กน้อยเพื่อให้กล้องและ IMU พร้อมก่อน
        TimerAction(period=3.0, actions=[rtabmap]),
    ])