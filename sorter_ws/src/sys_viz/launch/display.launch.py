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
            'enable_depth': 'true',
            'pointcloud.enable': 'true',
        }.items()
    )

def get_ur_driver_launch():
    # ur_with_EE_path = os.path.join(
    #     get_package_share_directory('robot_description'),
    #     'urdf',
    #     'ur_with_end_effector.xacro'
    # )
    ur_control_launch_path = os.path.join(get_package_share_directory('ur_robot_driver'),'launch','ur_control.launch.py')
    
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_control_launch_path),
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

    ur_moveit_octomap_launch_path = os.path.join(
        get_package_share_directory('moveit_config'),
        'launch;,'
        'ur_moveit_octomap.launch.py'
    )

    return TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_moveit_octomap_launch_path),
                launch_arguments={
                    'robot_ip': '192.168.0.100',
                    'ur_type': 'ur5e',
                    'launch_rviz': 'true',
                    'description_file': end_effector_path,
                    'moveit_config_package': 'moveit_config',
                }.items()
            )
        ]
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
        get_realsense_launch(),
        get_ur_driver_launch(),
        get_moveit_launch(),
        get_auxiliary_launch()
    ]
    return LaunchDescription(launch_description)