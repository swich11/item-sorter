import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


# launch arguments, change manually before launching
has_camera = 'false'
real_robot = 'false'

def get_realsense_launch():
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'enable_rgbd': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'pointcloud.enable': 'true',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '5',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '5',
            'pointcloud_texture_stream': 'RS2_STREAM_COLOR',
            'pointcloud_texture_index': '0',
            'filters': 'pointcloud',
            'allow_no_texture_points': 'false'
        }.items()
    )

def get_rviz_launch():
    rviz_config_path = os.path.join(
        get_package_share_directory('sys_viz'),
        'rviz',
        'display.rviz'
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_config_path)
    )



def generate_launch_description():
    launch_description = []
    return LaunchDescription(launch_description)