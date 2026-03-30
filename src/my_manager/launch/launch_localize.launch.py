import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():
    # --- 0. Config & Paths ---
    package_name = 'my_manager' 
    home_directory = os.path.expanduser('~')
    
    # กำหนด Path พื้นฐานสำหรับ Maps
    # แนะนำ: ใช้ PathJoinSubstitution จะปลอดภัยกว่าการต่อ String เองใน Python
    src_maps_path = os.path.join(home_directory, 'Robotics_Project/src/my_manager/maps')

    # --- 1. Arguments ---
    floor_arg = DeclareLaunchArgument(
        'floor', default_value='floor1',
        description='ชื่อโฟลเดอร์ชั้น'
    )
    db_name_arg = DeclareLaunchArgument(
        'db_name', default_value='mapping_run',
        description='ชื่อไฟล์ database'
    )

    # ดึงค่าจาก Argument
    floor_val = LaunchConfiguration('floor')
    db_name_val = LaunchConfiguration('db_name')

    # FIXED: ใช้ PythonExpression ที่สะอาดขึ้น หรือใช้ PathJoinSubstitution
    # การต่อ Path ใน ROS2 Launch ต้องระวังเรื่อง "Substitution" objects
    database_full_path = PythonExpression([
        "'", src_maps_path, "/' + '", floor_val, "' + '/' + '", db_name_val, "' + '.db'"
    ])

    # --- 2. Included Launch Files ---
    
    # Realsense
    # realsense = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
    #     )]),
    #     launch_arguments={
    #         'depth_module.profile': '640,480,15',
    #         'rgb_module.profile': '640,480,15',
    #         'pointcloud.enable': 'true',
    #         'align_depth.enable': 'true',
    #         'enable_gyro': 'true',
    #         'enable_accel': 'true',
    #         'unite_imu_method': '2',
    #         'enable_sync': 'true',
    #     }.items()
    # )

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('my_control'), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    # Joint State Publisher
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )

    # RViz Config Path
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'localized.rviz'
    )

    # --- 3. RTAB-Map (Mapping Mode) ---
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'database_path': database_full_path,
            'localization': 'true',
            'args': (
                '--RGBD/NeighborLinkRefining true '
                '--Vis/MinInliers 15 '              # ลองลดลงเหลือ 15 ชั่วคราวก่อนเพื่อให้หาเจอง่ายขึ้น
                '--RGBD/OptimizeMaxError 0 '        # ตั้งเป็น 0 คือปิดการจำกัด Error เพื่อให้มันยอมวาร์ป
                '--RGBD/ResetPoseOnLost true '      # ถ้าหลงทางให้รีเซ็ตตำแหน่ง
                '--Reg/Force3DoF true '
                '--Optimizer/Slam2d true '
                '--Mem/IncrementalMemory false'     # บังคับโหมด Localization บริสุทธิ์
            ),
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.2',
            'frame_id': 'base_footprint',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',           
            'odom_topic': '/odom/filtered',   
            'imu_topic': '/imu/data_standard',
            'odom_frame_id': 'odom',
            'publish_tf_map': 'true',
            'wait_imu_to_init': 'false',
            'wait_for_transform': '1.0', #
            'qos': '1',
            'rtabmap_viz': 'false',
            'rviz': 'true',
            'rviz_cfg': rviz_config_path,
        }.items()
    )

    # --- 4. Launch Description Assembly ---
    return LaunchDescription([
        floor_arg,
        db_name_arg,
        
        # รัน Node พื้นฐานทันที
        # realsense,
        rsp,
        node_joint_state_publisher,
        
        # หน่วงเวลา RTAB-Map 5 วินาทีเพื่อให้ Camera/TF พร้อมก่อน
        TimerAction(period=5.0, actions=[rtabmap]),
    ])