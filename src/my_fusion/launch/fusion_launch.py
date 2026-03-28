import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory('my_manager'), # เปลี่ยนเป็นชื่อ package คุณ
        'config',
        'twist_mux.yaml'
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[config_path],
        remappings=[('/cmd_vel_out', '/cmd_vel')] # ส่ง Output ออกไปที่หุ่นยนต์จริง
    )

    package_name = 'my_fusion'

    esp32_manager_node = Node(
        package='my_manager',
        executable='esp32_manager',
        name='esp32_manager_node',
        output='screen'
    )


    encoders_node = Node(
        package=package_name,
        executable='encoder_to_odom',
        name='encoder_to_odom_node',
        output='screen'
    )
    
    # 2. โหนดรับค่า IMU จาก ESP32
    imu_node = Node(
        package=package_name,
        executable='imu_bridge',
        name='imu_bridge_node',
        output='screen'
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'ekf_launch.py'
        )]), 
        launch_arguments={'use_sim_time': 'false'}.items()
    )

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
        ekf_launch,
        pid_node,
        twist_mux_node
    ])