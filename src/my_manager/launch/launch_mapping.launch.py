import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():
    # เปลี่ยนเป็น my_manager ให้ตรงกับที่ใช้รันจริง
    package_name = 'my_manager' 

    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'localized.rviz'
    )

    # --- 1. Arguments สำหรับเลือก Floor และชื่อไฟล์ ---
    floor_arg = DeclareLaunchArgument(
        'floor', default_value='floor1',
        description='ชื่อโฟลเดอร์ชั้น'
    )
    db_name_arg = DeclareLaunchArgument(
        'db_name', default_value='go',
        description='ชื่อไฟล์ db (ไม่ต้องใส่ .db)'
    )

    # FIXED: Corrected indentation here
    home_directory = os.path.expanduser('~')
    src_maps_path = os.path.join(
        home_directory, 
        'Robotics_Project/src/my_manager/maps'
    )

    # FIXED: Corrected indentation here
    # สร้าง Path แบบ Dynamic ไปที่ src
    database_full_path = PythonExpression([
        f"'{src_maps_path}/' + '", LaunchConfiguration('floor'), 
        f"/' + '", LaunchConfiguration('db_name'), ".db'"
    ])

    # --- 2. Nodes ต่างๆ ---
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
        )]),
        launch_arguments={
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'enable_sync': 'true',
            'initial_reset': 'true', 
            'depth_module.depth_visualization': 'true',
        }.items()
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

    # --- 3. RTAB-Map Localization ---
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'database_path': database_full_path,
            'localization': 'false', 
            'args': '--delete_db_on_start', 
            'rtabmap_args': (
                '--Reg/Strategy 0 '
                '--RGBD/NeighborLinkRefining true '
                '--Vis/MinInliers 12 '
                '--Rtabmap/DetectionRate 1'
                '--Mem/IncrementalMemory true '
            ),
            'approx_sync': 'true',
            'frame_id': 'base_link',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'odom_topic': '/odometry/filtered',   
            'imu_topic': '/imu/data_standard',   
            'wait_imu_to_init': 'true',
            'qos': '1',
            'rviz': 'true',
            'rviz_cfg': rviz_config_path ,
        }.items()
    )

    return LaunchDescription([
        floor_arg,
        db_name_arg,
        realsense,
        rsp,
        node_joint_state_publisher,
        TimerAction(period=3.0, actions=[rtabmap]),
    ])