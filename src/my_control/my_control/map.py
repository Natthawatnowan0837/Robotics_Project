import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ชื่อ package ของคุณ
    package_name = 'my_control'

    # ระบุ Path ไปยังไฟล์ map.yaml ในโฟลเดอร์ maps/floor1
    map_file_path = os.path.join(
        get_package_share_directory(package_name),
        'maps',
        'floor1',
        'map.yaml'
    )

    # Node สำหรับจัดการ Map (Map Server)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path},
                    {'use_sim_time': True}]
    )

    # Lifecycle Manager เพื่อเริ่มการทำงานของ map_server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    return LaunchDescription([
        map_server_node,
        lifecycle_manager_node
    ])