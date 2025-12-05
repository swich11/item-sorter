import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def get_display_launch():
    moveit_display_launch_path = os.path.join(
        get_package_share_directory('sys_viz'),
        'launch',
        'display.launch.py'
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_display_launch_path)
    )

def get_auxiliary_launch():
    auxiliary_launch_path = os.path.join(
        get_package_share_directory('sys_viz'),
        'launch',
        'auxiliary.launch.py'
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(auxiliary_launch_path)
    )

def generate_launch_description():
    launch_description = [
        get_display_launch(),
        get_auxiliary_launch(),
    ]
    return LaunchDescription(launch_description)