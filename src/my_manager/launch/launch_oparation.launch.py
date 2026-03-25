import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # voice_command_path = get_package_share_directory('my_voice_control')
    # launch_voice_command = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(voice_command_path, 'launch', 'launch_voice_control.launch.py')
    #     )
    # )



#--------------------------------------------------------------------------

    my_manager_run = Node(
        package='my_manager',
        executable='my_manager',
        name='my_manager_node',
        output='screen'
    )

    open_map = Node(
        package='my_manager',
        executable='open_map',
        name='open_map_node',
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

    check_position = Node(
        package='my_manager',
        executable='check_position',
        name='check_position_node',
        output='screen'
    )
        
    nav2 = Node(
        package='my_manager',
        executable='nav_goal',
        name='nav2_node',
        output='screen'
    )

    rotation_control = Node(
        package='my_manager',
        executable='rotation_control',
        name='rotation_control_node',
        output='screen'
    )
#--------------------------------------------------------------------------

    return LaunchDescription([
        # launch_voice_command,
        my_manager_run,
        open_map,
        check_floor,
        check_localize,
        check_position,
        rotation_control,
        nav2
    ])