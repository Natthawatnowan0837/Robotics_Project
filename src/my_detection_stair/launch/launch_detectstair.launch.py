import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_detection_stair'

    # ดึง Path ของไฟล์ .rviz (เปลี่ยนชื่อไฟล์ให้ตรงกับที่อยู่ในโฟลเดอร์ rviz ของคุณ)
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'detection.rviz' # <--- เปลี่ยนเป็นชื่อไฟล์จริงของคุณ
    )

    stair_detection_node = Node(
        package=package_name,
        executable='test_model',
        name='stair_inference_node',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        stair_detection_node,
        rviz_node
    ])