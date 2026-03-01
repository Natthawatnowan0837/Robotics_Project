import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_control'

    # 1. Robot State Publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
             )

    # 3. Spawn Entity
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
                        output='screen')

    # 4. RViz2
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen')

    # 5. SLAM Toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
        )]), 
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 6. Static Transform Publisher
    # "IMU frame is missing" หรือตำแหน่ง IMU ไม่เชื่อมต่อกับตัวหุ่นยนต์ในแผนผังพิกัด
    static_transform_publisher = Node(
        package='tf2_ros', 
        executable='static_transform_publisher', 
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'chassis_link', 'camera_link'],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        rviz,
        slam_toolbox,
        static_transform_publisher,  # Add static transform publisher node here
    ])
