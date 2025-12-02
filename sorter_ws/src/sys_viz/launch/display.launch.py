import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def get_realsense_launch():
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true'
        }.items()
    )

def get_ur_driver_launch():
    # ur_with_EE_path = os.path.join(
    #     get_package_share_directory('robot_description'),
    #     'urdf',
    #     'ur_with_end_effector.xacro'
    # )

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
            #'decription_file': ur_with_EE_path
        }.items()
    )

def get_moveit_launch():
    end_effector_path = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'ur_with_end_effector.xacro'
    )

    return TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('moveit_config'),
                        'launch',
                        'ur_moveit.launch.py'
                    )
                ),
                launch_arguments={
                    'robot_ip': '192.168.0.100',
                    'ur_type': 'ur5e',
                    'launch_rviz': 'true'
                    #,
                    #'description_file': end_effector_path,
                }.items()
            )
        ]
    )

def get_rviz_launch():
    moveit_launch_path = os.path.join(
        get_package_share_directory('moveit_config'), 'launch', 'ur_moveit_rviz.launch.py'
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_path)
    )

def get_moveit_planner_launch():
    moveit_planner_launch_path = os.path.join(
        get_package_share_directory('moveit_planner'),
        'launch',
        'planner.launch.py'
    )

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_planner_launch_path)
    )

def generate_launch_description():
    launch_description = [
        get_realsense_launch(),
        get_ur_driver_launch(),
        get_moveit_launch(),
        get_moveit_planner_launch(),
    ]
    return LaunchDescription(launch_description)