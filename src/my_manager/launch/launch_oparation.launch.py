import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # ดึง Path ของ package เสียง (Uncomment เพื่อใช้งาน)
    # voice_command_path = get_package_share_directory('my_voice_control')
    # launch_voice_command = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(voice_command_path, 'launch', 'launch_voice_control.launch.py')
    #     )
    # )

#--------------------------------------------------------------------------
# ส่วนของ Node ต่างๆ ใน Package my_manager

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

    goal = Node(
        package='my_manager',
        executable='goal',
        name='goal_node',
        output='screen'
    )
        
    nav2 = Node(
        package='my_manager',
        executable='nav2',
        name='nav2_node',
        output='screen'
    )

    rotation_control = Node(
        package='my_manager',
        executable='rotation_control',
        name='rotation_control_node',
        output='screen'
    )

    # Node สำหรับไปที่บันได
    goto_stair_node = Node(
        package='my_manager',
        executable='go_to_stair',
        name='goto_stair_node',
        output='screen'
    )

    # Node สำหรับ Keyboard Controller (ที่รับ Service)
    controller_node = Node(
        package='my_manager',
        executable='controller',
        name='controller_node',
        output='screen',
        emulate_tty=True # สำคัญ: เพื่อให้แสดงผลและรับค่าคีย์บอร์ดใน Terminal ได้ดีขึ้น
    )

#--------------------------------------------------------------------------

    return LaunchDescription([
        # ระบบเสียง
        # launch_voice_command,
        
        # ระบบจัดการหลัก
        my_manager_run,
        open_map,
        check_floor,
        check_localize,
        check_position,
        rotation_control,
        nav2,
        
        # ระบบเป้าหมายและการเคลื่อนที่พิเศษ
        goal,
        goto_stair_node,
        controller_node
    ])