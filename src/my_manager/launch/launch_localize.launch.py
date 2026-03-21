import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():
    package_name = 'my_manager'

    # Path สำหรับ RViz Config
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'localized.rviz'
    )

    # --- 1. จัดการ Arguments และ Database Path ---
    
    # ประกาศ Argument "floor"
    floor_arg = DeclareLaunchArgument(
        'floor',
        default_value='floor1',
        description='ชื่อโฟลเดอร์ชั้น (เช่น floor1, floor2)'
    )

    # ตัวแปรเลือกชื่อไฟล์ db (เช่น go, back)
    db_name_arg = DeclareLaunchArgument(
        'db_name',
        default_value='go',
        description='ชื่อไฟล์ database (เช่น go, back) โดยไม่ต้องใส่ .db'
    )

    # สร้าง Path: share/my_control/maps/<floor>/<db_name>.db
    database_full_path = PathJoinSubstitution([
        get_package_share_directory('my_manager'),
        'maps',
        LaunchConfiguration('floor'),
        PythonExpression(["'", LaunchConfiguration('db_name'), ".db'"])
    ])

    # --- 2. ตั้งค่า Nodes ต่างๆ ---

    # Realsense
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
            'initial_reset': 'true', # แก้ปัญหา Device Busy
            'depth_module.depth_visualization': 'true',
        }.items()
    )

    # Robot State Publisher
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

    # --- 3. RTAB-Map Localization ---
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'database_path': database_full_path,
            'localization': 'true',
            'rtabmap_args': (
                '--Mem/IncrementalMemory false '
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

    return LaunchDescription([
        floor_arg,      # แก้ให้ชื่อตรงกับตอน Declare
        db_name_arg,    # เพิ่มเข้าไป
        realsense,      # เพิ่มเข้าไปเพื่อให้มี odom
        rsp,
        node_joint_state_publisher,
        TimerAction(period=3.0, actions=[rtabmap]),
    ])