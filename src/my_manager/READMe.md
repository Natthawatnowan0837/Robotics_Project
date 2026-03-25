ros2 launch my_manager launch_mapping.launch.py floor:=floor2 db_name:=go
ros2 launch my_manager launch_localize.launch.py floor:=floor2 db_name:=back
ros2 topic echo /rtabmap/localization_pose
ros2 topic pub --once /room_target std_msgs/msg/String "{data: 'A1'}"

noone@noone-HP-ProDesk-400-G4-DM:~/Robotics_Project$ ros2 run my_manager my_manager 
[INFO] [1774392857.503342172] [state_manager_node]: 📚 Loaded 18 rooms.
[INFO] [1774392857.511124352] [state_manager_node]: 🤖 State Manager Ready. Waiting for order...
[INFO] [1774393122.490986048] [state_manager_node]: 🔎 Target: B2 | Way: back | Goal: [0.0, 0.0]
[INFO] [1774393122.511911256] [state_manager_node]: 🔄 Current State: CHECK_FLOOR
[INFO] [1774393122.512443328] [state_manager_node]: 📡 Calling Service for floor: 3.0
[INFO] [1774393122.515566527] [state_manager_node]: 🏢 Current Floor: 2.0 | Status: Up
[WARN] [1774393122.516086041] [state_manager_node]: 🔄 Floor Mismatch! Switching target to: Up_Stair2
[INFO] [1774393123.514057328] [state_manager_node]: 🔄 Current State: OPEN_MAP
[INFO] [1774393123.515796517] [state_manager_node]: 📡 Calling OpenMap: Mode=localize, Way=back, Floor=2.0
INFO] [1774393202.512620815] [state_manager_node]: 🔄 Current State: CHECK_POSITION
[INFO] [1774393202.513558413] [state_manager_node]: 📡 Sending Position: X=29.84, Y=9.19, Way=back
[INFO] [1774393202.517907709] [state_manager_node]: ✅ Position Confirmed. Updated Way: back
[INFO] [1774393202.518311045] [state_manager_node]: 🚀 Destination confirmed. Starting Navigation...
[INFO] [1774393203.511511361] [state_manager_node]: 🔄 Current State: NAV2
[ERROR] [1774393203.511925962] [state_manager_node]: ❌ Goal is [0, 0]. Moving to IDLE.
