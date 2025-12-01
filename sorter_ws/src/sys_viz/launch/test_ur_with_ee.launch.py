import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    end_effector_path = os.path.join(
        get_package_share_directory('robot_description'), 'urdf', 'ur_with_end_effector.xacro'
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py"
            ])
        ),
        launch_arguments={
            # OVERRIDE THE DEFAULT URDF HERE ↓↓↓
            "description_file": end_effector_path,
            #"description_package": "robot_description",   # your package where xacro lives
            "ur_type": "ur5e",
            "prefix": "",
            "launch_rviz": "true"
        }.items(),
    )

    return LaunchDescription([moveit_launch])
