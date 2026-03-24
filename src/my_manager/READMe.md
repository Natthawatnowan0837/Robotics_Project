ros2 launch my_manager launch_mapping.launch.py floor:=floor2 db_name:=go
ros2 launch my_manager launch_localize.launch.py floor:=floor2 db_name:=back
ros2 topic echo /rtabmap/localization_pose