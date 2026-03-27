import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_fusion'

    # 1. โหนดจัดการการเชื่อมต่อ ESP32
    esp32_manager_node = Node(
        package='my_manager',
        executable='esp32_manager',
        name='esp32_manager_node',
        output='screen'
    )

    # 2. โหนดแปลงค่า Encoder เป็น Odometry
    encoders_node = Node(
        package=package_name,
        executable='encoder_to_odom',
        name='encoder_to_odom_node',
        output='screen'
    )
    
    # 3. โหนดรับค่า IMU จาก ESP32
    imu_node = Node(
        package=package_name,
        executable='imu_bridge',
        name='imu_bridge_node',
        output='screen'
    )

    # 4. ดึง EKF Launch (Extended Kalman Filter)
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'ekf_launch.py'
        )]), 
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # 5. ดึง Odom Launch
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'odom_launch.py'
        )]), 
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # 6. โหนดควบคุม PID
    pid_node = Node(
        package='my_control',
        executable='pid_parameters',
        name='pid_node',
        output='screen'
    )

    # --- ส่งโหนดทั้งหมดออกไปรัน ---
    return LaunchDescription([
        esp32_manager_node,
        encoders_node,
        imu_node,
        odom_launch,  # <--- เติมจุลภาคตรงนี้
        ekf_launch,   # <--- เติมจุลภาคตรงนี้
        pid_node      # <--- เติมจุลภาคตรงนี้
    ])