import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource





def get_ur_sim_launch():
    description_file = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'ur_with_end_effector.xacro'
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_robot_driver'),'launch','ur_control.launch.py'
            )
        ),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': '192.168.0.100',
            'use_fake_hardware': 'false',
            'launch_rviz': 'false',
            'use_fake_hardware': 'true',
            'description_file': description_file,
        }.items()
    )


def generate_launch_description():
    launch_description = [
        get_ur_sim_launch(),
    ]
    return LaunchDescription(launch_description)