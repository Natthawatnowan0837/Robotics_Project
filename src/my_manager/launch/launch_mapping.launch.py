import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

def generate_launch_description():
    # --- 0. Config & Paths ---
    package_name = 'my_manager' 
    home_directory = os.path.expanduser('~')
    
    # Path สำหรับเก็บแผนที่
    src_maps_path = os.path.join(home_directory, 'Robotics_Project/src/my_manager/maps')

    apriltag_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'apriltag.yaml'
    )
    
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'localized.rviz'
    )

    # --- 1. Arguments ---
    floor_arg = DeclareLaunchArgument(
        'floor', default_value='floor1',
        description='ชื่อโฟลเดอร์ชั้น'
    )
    db_name_arg = DeclareLaunchArgument(
        'db_name', default_value='mapping_run',
        description='ชื่อไฟล์ database'
    )

        # FIXED: Corrected indentation here
    home_directory = os.path.expanduser('~')
    src_maps_path = os.path.join(
        home_directory, 
        'Robotics_Project/src/my_manager/maps'
    )

    # FIXED: แก้ไข PythonExpression ให้รับค่า String จาก LaunchConfiguration อย่างถูกต้อง
    database_full_path = PythonExpression([
        f"'{src_maps_path}/' + '", LaunchConfiguration('floor'), 
        f"/' + '", LaunchConfiguration('db_name'), ".db'"
    ])


    # # AprilTag
    # apriltag_node = Node(
    #     package='apriltag_ros',
    #     executable='apriltag_node',
    #     name='apriltag_node',
    #     parameters=[apriltag_config],
    #     remappings=[
    #         ('image_rect', '/camera/camera/color/image_raw'),
    #         ('camera_info', '/camera/camera/color/camera_info'),
    #         ('detections', '/detections')
    #     ]
    # )

    # --- 3. RTAB-Map (Mapping Mode) ---
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'database_path': database_full_path,
            'localization': 'false',
            'args': (
                '--delete_db_on_start '
                '--Mem/IncrementalMemory true '
                '--RGBD/NeighborLinkRefining true '
                '--Vis/MinInliers 15 '
                # '--RGBD/TagHasConstantSize true '
                # '--Landmarks/FromTags true '
                '--RGBD/OptimizeMaxError 10.0 '
                '--Rtabmap/DetectionRate 1 '
            ),
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.05',
            'frame_id': 'base_footprint',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'tag_topic': '/detections',            
            'odom_topic': '/odometry/filtered',   
            'imu_topic': '/imu/data_standard',
            'odom_frame_id': 'odom',
            'publish_tf_map': 'true',
            'wait_imu_to_init': 'true',
            'qos': '1',
            'rviz': 'true',
            'rviz_cfg': rviz_config_path,
        }.items()
    )

    return LaunchDescription([
        floor_arg,
        db_name_arg,
        # apriltag_node,
        TimerAction(period=5.0, actions=[rtabmap]),
    ])