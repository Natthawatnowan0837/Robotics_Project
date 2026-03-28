ros2 launch my_manager launch_mapping.launch.py floor:=floor2 db_name:=go
ros2 launch my_manager launch_localize.launch.py floor:=floor2 db_name:=back
ros2 topic echo /rtabmap/localization_pose
ros2 topic pub --once /room_target std_msgs/msg/String "{data: 'A1'}"

pkill -9 -f ros2
pkill -9 -f micro_ros_agent
sudo rm -rf /dev/shm/fastrtps*
ros2 daemon stop
ros2 daemon start