from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Path to the standard UR MoveIt2 launch
    ur_moveit_config_dir = get_package_share_directory('ur_moveit_config')

    # Include the default UR MoveIt launch file
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ur_moveit_config_dir, 'launch', 'ur_moveit.launch.py'])
        )
    )

    # Define OctoMap monitor parameters
    octomap_params = {
        'octomap_frame': 'base_link',
        'pointcloud_topic': '/camera/camera/depth/color/points',
        'octomap_resolution': 0.05,
        'max_range': 2.0,

        'use_sim_time': False,
    }

    # Add the Occupancy Map Monitor node
    octomap_monitor = Node(
        package='moveit_ros_occupancy_map_monitor',
        executable='occupancy_map_monitor',
        name='occupancy_map_monitor',
        output='screen',
        parameters=[octomap_params],
    )

    return LaunchDescription([
        ur_moveit_launch,
        octomap_monitor
    ])
