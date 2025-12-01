import launch

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # start brain node
    brain_node = Node(
        package="brain",
        executable="brain",
        name="brain"
    ),
    arduino_bridge_node = Node(
        package="teensy_pkg",
        executable="util_arduino_node",
        name="util_arduino_node",
    ),
    transform_node = Node(
        package="transforms",
        executable="PoseLookup",
        name="PoseLookup",
    )
    #,
    # object_detector_node = Node(
    #     package="object_detect",
    #     executable="object_detector",
    #     name="object_detect",
    # )



    return launch.LaunchDescription([
        brain_node,
        arduino_bridge_node,
        transform_node,
        #object_detector_node
    ])