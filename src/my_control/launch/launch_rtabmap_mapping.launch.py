import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # --- 0. Config & Paths ---
    package_name = 'my_manager' 
    home_directory = os.path.expanduser('~')
    
    # กำหนด Path สำหรับ Maps
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

    floor_val = LaunchConfiguration('floor')
    db_name_val = LaunchConfiguration('db_name')

    # ใช้ PathJoinSubstitution จะปลอดภัยกว่า PythonExpression ในกรณีนี้
    database_full_path = PathJoinSubstitution([
        src_maps_path, floor_val, [db_name_val, ".db"]
    ])

    # --- 2. Included Launch Files ---
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
            'localization': 'false',
            # FIXED: ต้องส่งเป็น String ยาวๆ ต่อกัน (ใช้ช่องว่างคั่น) ห้ามมีคอมม่าคั่นระหว่างบรรทัด
            'args': (
                '--Mem/IncrementalMemory true '
                '--RGBD/NeighborLinkRefining true '
                '--Vis/MinInliers 15 '
                '--RGBD/OptimizeMaxError 10.0 '
                '--Rtabmap/DetectionRate 1 '
                '--Reg/Force3DoF true '
                '--Optimizer/Slam2d true'
            ),
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.2', 
            'frame_id': 'base_footprint',
            
            # --- ส่วนการใช้งาน Compressed ---
            'subscribe_rgb': 'true',
            'subscribe_depth': 'true',
            # ใส่ชื่อ Topic หลัก (Base topic) แล้ว RTAB-Map จะไปหา /compressed ต่อเอง
            'rgb_topic': '/camera/camera/color/image_raw',       
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw', 
            'rgb_image_transport': 'compressed',                
            'depth_image_transport': 'compressedDepth',         
            # -------------------------------

            'camera_info_topic': '/camera/camera/color/camera_info',          
            'odom_topic': '/odom/filtered', 
            'imu_topic': '/imu/data_standard',
            'odom_frame_id': 'odom',
            'publish_tf_map': 'true',
            'wait_imu_to_init': 'false',
            'wait_for_transform': '1.5', # เพิ่มนิดหน่อยเผื่อ CPU โหลดหนักตอนแตกไฟล์ภาพ
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
        rsp,
        node_joint_state_publisher,
        TimerAction(period=5.0, actions=[rtabmap]),
    ])