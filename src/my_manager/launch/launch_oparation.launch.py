import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    voice_command_path = get_package_share_directory('my_voice_control')
    launch_voice_command = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(voice_command_path, 'launch', 'launch_voice_control.launch.py')
        )
    )

#--------------------------------------------------------------------------
    esp32_manager = Node(
        package='my_manager',
        executable='esp32_manager',
        name='esp32_manager_node',
        output='screen'
    )

    my_manager_run = Node(
        package='my_manager',
        executable='my_manager',
        name='my_manager_node',
        output='screen'
    )

    select_map = Node(
        package='my_manager',
        executable='select_map',
        name='select_map_node',
        output='screen'
    )
    

    check_floor = Node(
        package='my_manager',
        executable='check_floor',
        name='check_floor_node',
        output='screen'
    )

    check_localize = Node(
        package='my_manager',
        executable='check_localize',
        name='check_localize_node',
        output='screen'
    )
#--------------------------------------------------------------------------
    fusion_sensors_path = get_package_share_directory('my_fusion')
    launch_fusion_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fusion_sensors_path, 'launch', 'launch_fusion.launch.py')
        )
    )

    return LaunchDescription([
        launch_voice_command,
        esp32_manager,
        my_manager_run,
        select_map,
        check_floor,
        check_localize,
        launch_fusion_sensors
    ])