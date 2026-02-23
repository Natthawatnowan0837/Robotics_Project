import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- Node ตัวที่ 1: รันแบบเจาะจงค่า (Manual Localization) ---
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
                'localization': True,
                'database_path': os.path.expanduser('~/Documents/office_map.db'),
                'subscribe_depth': True,
                'approx_sync': True,
                'use_action_for_goal': True,
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true'
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info')
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ), # <--- ใส่คอมมาเชื่อมระหว่าง Node

        # --- Node ตัวที่ 2: รันแบบ Dynamic (ใช้ LaunchConfiguration) ---
        Node(
            package='rtabmap_slam', executable='rtabmap', name="rtabmap_dynamic", output="screen",
            emulate_tty=True,
            parameters=[{
                "subscribe_depth": LaunchConfiguration('depth', default='true'),
                "subscribe_rgbd": LaunchConfiguration('subscribe_rgbd', default='false'),
                "subscribe_rgb": LaunchConfiguration('subscribe_rgb', default='true'),
                "subscribe_stereo": LaunchConfiguration('stereo', default='false'),
                "subscribe_scan": LaunchConfiguration('subscribe_scan', default='false'),
                "subscribe_scan_cloud": LaunchConfiguration('subscribe_scan_cloud', default='false'),
                "subscribe_user_data": LaunchConfiguration('subscribe_user_data', default='false'),
                
                # แก้ไข Conditional Logic สำหรับ subscribe_odom_info
                "subscribe_odom_info": PythonExpression([
                    "'true' if '", LaunchConfiguration('icp_odometry', default='false'), 
                    "' == 'true' or '", LaunchConfiguration('visual_odometry', default='false'), "' == 'true' else 'false'"
                ]),

                "frame_id": LaunchConfiguration('frame_id', default='base_link'),
                "map_frame_id": LaunchConfiguration('map_frame_id', default='map'),
                "odom_frame_id": LaunchConfiguration('odom_frame_id', default='odom'),
                "publish_tf": LaunchConfiguration('publish_tf_map', default='true'),
                "use_action_for_goal": LaunchConfiguration('use_action_for_goal', default='true'),
                "database_path": LaunchConfiguration('database_path', default='~/.ros/rtabmap.db'),
                "approx_sync": LaunchConfiguration('approx_sync', default='true'),
                
                # แก้ไข Mem/IncrementalMemory (Mapping vs Localization)
                "Mem/IncrementalMemory": PythonExpression([
                    "'false' if '", LaunchConfiguration('localization', default='false'), "' == 'true' else 'true'"
                ]),
                "Mem/InitWMWithAllNodes": LaunchConfiguration('localization', default='false'),
            }],
            remappings=[
                ("rgb/image", LaunchConfiguration('rgb_topic', default='/camera/color/image_raw')),
                ("depth/image", LaunchConfiguration('depth_topic', default='/camera/aligned_depth_to_color/image_raw')),
                ("rgb/camera_info", LaunchConfiguration('camera_info_topic', default='/camera/color/camera_info')),
                ("odom", LaunchConfiguration('odom_topic', default='/odom'))
            ],
            # ปรับปรุงส่วน arguments ให้รองรับ log level พื้นฐาน
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level', default='info')]
        )
    ])