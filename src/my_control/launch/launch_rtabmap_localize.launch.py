import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'my_control'

    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'localized.rviz'
    )

    # 1. Realsense Camera
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
            'depth_module.depth_visualization': 'true',
        }.items()
    )

    # 2. Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('my_sim'), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )

    # 3. IMU Filter
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


    pid_parameters_node =  Node(
            package='my_control', # ชื่อ package
            executable='pid_parameters_node', # ชื่อที่ตั้งไว้ใน setup.py หรือชื่อไฟล์
            name='pid_parameters_node', # ชื่อ node ตอนรัน (optional)
            output='screen' # ให้แสดง log บนหน้าจอ terminal
    )

    # 4. RTAB-Map Configuration
    database_full_path = os.path.expanduser('/home/noone/Robotics_Project/src/my_control/map/floor1/back/back.db')
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'database_path': database_full_path,
            'localization': 'true', 
            'rtabmap_args': '--Mem/IncrementalMemory false',
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

    return LaunchDescription([
        realsense,
        rsp,
        node_joint_state_publisher,
        imu_filter,
        pid_parameters_node,
        
        # รัน RTAB-Map หลังผ่านไป 3 วินาที
        TimerAction(period=3.0, actions=[rtabmap]),
    ])