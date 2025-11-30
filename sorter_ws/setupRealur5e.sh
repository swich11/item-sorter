gnome-terminal -t "DepthCamera" -e 'ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_color:=true enable_depth:= true pointcloud.enable:=true'


gnome-terminal -t "DriverServer" -e 'ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 use_fake_hardware:=false launch_rviz:=false'

sleep 10

gnome-terminal -t "MoveitServer" -e 'ros2 launch moveit_config ur_moveit_octomap.launch.py robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true moveit_config_package:=moveit_config'





