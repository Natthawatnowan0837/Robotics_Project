import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. โหนดสำหรับปรับแต่งค่า PID (Parameter Node)
    pid_parameters_node = Node(
        package='my_control',
        executable='pid_parameters_node',
        name='pid_parameters_node',
        output='screen'
    )
    
    # 2. โหนดสำหรับประมวลผลการควบคุม (Controller Node)
    controller = Node(
        package='my_control',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )

    return LaunchDescription([
        pid_parameters_node,
        controller
    ])