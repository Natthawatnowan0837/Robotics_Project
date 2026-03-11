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
        'localize.rviz'
    )
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

    # controller =  Node(
    #         package='my_control', # ชื่อ package
    #         executable='xbox_controller_node', # ชื่อที่ตั้งไว้ใน setup.py หรือชื่อไฟล์
    #         name='xbox_controller_node', # ชื่อ node ตอนรัน (optional)
    #         parameters=[{'mode': 'map'}], # ส่ง parameter (optional)
    #         output='screen' # ให้แสดง log บนหน้าจอ terminal
    # )

    # เพิ่ม .db ต่อท้ายชื่อไฟล์ที่ต้องการบันทึก
    database_full_path = os.path.expanduser('/home/noone/Robotics_Project/src/my_control/map/floor1/back/back.db')

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'database_path': database_full_path,  # <--- ต้องใส่บรรทัดนี้ครับ
            'localization': 'true', 
            'rtabmap_args': '--Mem/IncrementalMemory false', # บังคับโหมดอ่านอย่างเดียว
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'base_link',
            'approx_sync': 'true', 
            'imu_topic': '/rtabmap/imu',
            'wait_imu_to_init': 'true',
            'qos': '1',
            'rviz': 'true',
            'rviz_cfg': rviz_config_path 
        }.items()
    )



    # nav2_params_path = os.path.join(
    #     get_package_share_directory(package_name), 'config', 'nav2_params.yaml')

    # navigation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
    #     )]),
    #     launch_arguments={
    #         'use_sim_time': 'false',
    #         'params_file': nav2_params_path,
    #         'autostart': 'true',
    #         'use_amcl': 'false',
    #         'map': '/rtabmap/map',
    #             # <--- เพิ่มบรรทัดนี้เพื่อปิด AMCL
    #     }.items()
    # )

    return LaunchDescription([
        realsense,
        rsp,
        node_joint_state_publisher,
        imu_filter,
        # controller,
        # navigation_launch,
        # static_tf,
        # แนะนำให้ใช้ TimerAction หน่วงเวลา RTAB-Map เล็กน้อยเพื่อให้กล้องและ IMU พร้อมก่อน
        TimerAction(period=3.0, actions=[rtabmap]),
        # TimerAction(period=15.0, actions=[navigation]),
    ])