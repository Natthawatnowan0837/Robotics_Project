pip install open3d
ros2 run my_detection_stair point_cloud_processor --ros-args -p class_label:=upstairs

ros2 run my_detection_stair point_cloud_processor --ros-args -p class_label:=downstairs

ros2 run my_detection_stair point_cloud_processor --ros-args -p class_label:=others