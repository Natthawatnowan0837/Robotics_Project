import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ---------------------------------------------------
    # 1. กำหนดชื่อ Package และ Model ของเรา
    # ---------------------------------------------------
    pkg_name = 'my_control'
    robot_name = 'my_stair_robot'
    file_name = 'model.urdf'

    # ---------------------------------------------------
    # 2. รับค่า Configuration (เช่น เวลาจำลอง)
    # ---------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ---------------------------------------------------
    # 3. หาตำแหน่งไฟล์ Model (SDF)
    # ---------------------------------------------------
    pkg_share = get_package_share_directory(pkg_name)
    
    # Path จะเป็น: .../share/my_control/models/my_stair_robot/model.sdf
    model_path = os.path.join(pkg_share, 'models', robot_name, file_name)

    print('Model file path : {}'.format(model_path))

    # อ่านไฟล์ SDF เตรียมไว้ส่งให้ Node
    with open(model_path, 'r') as infp:
        robot_desc = infp.read()

    # ---------------------------------------------------
    # 4. สร้าง Launch Description
    # ---------------------------------------------------
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }],
        ),
    ])