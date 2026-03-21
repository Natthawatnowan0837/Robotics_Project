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
            'unite_imu_method': '2',
            'enable_sync': 'true',
            'depth_module.depth_visualization': 'true',
        }.items()
    )

    # 2. Robot State Publisher
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

    # 4. RTAB-Map Configuration
    database_full_path = os.path.expanduser('/home/noone/Robotics_Project/src/my_manager/maps/floor2/go.db')
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
        )]),
        launch_arguments={
            'database_path': database_full_path,
            'localization': 'true',                # เปิดโหมด Localization (ไม่อัปเดตแผนที่ใหม่)
            
                        # rtabmap_args สำหรับโหมด Localization
            'rtabmap_args': (
                            '--Mem/IncrementalMemory false '
                            '--Mem/InitMemoryWMS true '
                            '--RGBD/OptimizeFromGraphEnd false '
                            
                            # 1. เพิ่มความเร็วในการประมวลผลช่วงหาตำแหน่ง (แนะนำ 0.5 - 1.0 Hz)
                            '--Rtabmap/DetectionRate 0.8 '       
                            
                            # 2. เพิ่มจำนวนจุด Feature เพื่อให้จำ "ทางเดิน" ได้แม่นขึ้น (สำคัญมาก!)
                            '--Kp/MaxFeatures 800 '              
                            
                            # 3. บังคับให้ค้นหาจากภาพทั่วทั้ง Database (Global Localization)
                            '--RGBD/ProximityBySpace false '     
                            '--RGBD/LoopClosureRecheck true '    
                            
                            # 4. ช่วยให้การเชื่อมต่อตำแหน่งนิ่งขึ้น
                            '--RGBD/NeighborLinkRefining true '
                            '--Vis/MinInliers 15 '               # ต้องเจอจุดเหมือนกันอย่างน้อย 15 จุดถึงจะยอมรับตำแหน่ง
                            
                            # 5. ลดภาระการคำนวณส่วนอื่นเพื่อชดเชย CPU
                            '--RGBD/AngularUpdate 0.1 '          # หมุนนิดเดียวให้รีบเช็กตำแหน่ง
                            '--RGBD/LinearUpdate 0.1'            # ขยับนิดเดียวให้รีบเช็กตำแหน่ง
                        ),
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'base_link',
            
            # --- ตั้งค่าให้ใช้ข้อมูลจาก EKF (เหมือนโหมด Mapping) ---
            'visual_odometry': 'false',           # ปิด Visual Odom ของ rtabmap
            'odom_topic': '/odometry/filtered',   # ใช้ค่า Fusion (Wheel + IMU)
            'publish_tf_odom': 'false',           # ให้ EKF เป็นคนส่ง TF odom -> base_link
            'vo_frame_id': 'odom',
            
            # --- การตั้งค่าอื่นๆ ---
            'approx_sync': 'true', 
            'imu_topic': '/imu/data_standard',    # แก้ให้ตรงกับ Node ของคุณ
            'wait_imu_to_init': 'false',          # แนะนำเป็น false ถ้าใช้ EKF นำทางอยู่แล้ว
            'qos': '1',
            'rviz': 'true',
            'rviz_cfg': rviz_config_path 
        }.items()
    )

    return LaunchDescription([
        realsense,
        rsp,
        node_joint_state_publisher,
        TimerAction(period=3.0, actions=[rtabmap]),
    ])