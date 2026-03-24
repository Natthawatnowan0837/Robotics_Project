import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():
    # 1. กำหนดชื่อ Package
    package_name = 'my_manager'
    control_package = 'my_control'

    # 2. ตั้งค่า Path ภายนอก (ดึงจาก Home โดยตรง)
    # ใช้ os.path.expanduser('~') เพื่อให้ชี้ไปที่ /home/<username> อัตโนมัติ
    home_dir = os.path.expanduser('~')
    
    # แก้ไข Path ตรงนี้ให้ตรงกับที่อยู่ไฟล์ .db จริงของคุณ
    # ตัวอย่าง: /home/noone/Robotics_Project/src/my_manager/maps/
    maps_base_path = os.path.join(home_dir, 'Robotics_Project', 'src', 'my_manager', 'maps')

    # Path สำหรับ RViz Config (ดึงจาก share ปกติ)
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'localized.rviz'
    )

    # --- 3. จัดการ Arguments ---
    floor_arg = DeclareLaunchArgument(
        'floor',
        default_value='floor2',
        description='ชื่อโฟลเดอร์ชั้น (เช่น floor1, floor2)'
    )

    db_name_arg = DeclareLaunchArgument(
        'db_name',
        default_value='back',
        description='ชื่อไฟล์ database (เช่น go, back) โดยไม่ต้องใส่ .db'
    )

    # --- 4. สร้าง Dynamic Path สำหรับ Database ---
    # เชื่อมต่อ path: maps_base_path / floor / db_name.db
    database_full_path = PythonExpression([
        f"'{maps_base_path}/' + '", LaunchConfiguration('floor'), "/' + '", 
        LaunchConfiguration('db_name'), ".db'"
    ])

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
    # --- 5. ตั้งค่า Nodes ต่างๆ ---

    # Realsense Camera
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
            'initial_reset': 'true',
            'depth_module.depth_visualization': 'true',
        }.items()
    )

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(control_package), 'launch', 'rsp.launch.py'
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

    # --- 6. RTAB-Map Localization ---
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'database_path': database_full_path,  # ใช้ Path ที่เราสร้างขึ้นใหม่
            'localization': 'true',               # โหมดหาตำแหน่ง (ไม่สร้าง Map ใหม่)
            'rtabmap_args': (
                '--Mem/IncrementalMemory false '  # สำคัญมากสำหรับ Localization
                '--Mem/InitMemoryWMS true ' 
                '--Rtabmap/DetectionRate 0.8 '       
                '--Kp/MaxFeatures 800 '              
                '--RGBD/ProximityBySpace false '     
                '--RGBD/LoopClosureRecheck true '    
                '--RGBD/NeighborLinkRefining true '
                '--Vis/MinInliers 15 '               
                '--RGBD/AngularUpdate 0.1 '          
                '--RGBD/LinearUpdate 0.1'            
            ),
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'base_link',
            'visual_odometry': 'false',           
            'odom_topic': '/odometry/filtered',   
            'imu_topic': '/imu/data_standard',   
            'publish_tf_odom': 'false',           
            'vo_frame_id': 'odom',
            'approx_sync': 'true', 
            'wait_imu_to_init': 'false',          
            'qos': '1',
            'rviz': 'true',
            'rviz_cfg': rviz_config_path 
        }.items()
    )

    # --- 7. รวม Launch ทั้งหมด ---
    return LaunchDescription([
        floor_arg,
        db_name_arg,
        realsense,
        rsp,
        node_joint_state_publisher,
        imu_filter,
        # หน่วงเวลา rtabmap 3 วินาที เพื่อให้ Sensor และ TF พร้อมก่อน
        TimerAction(period=3.0, actions=[rtabmap]),
    ])