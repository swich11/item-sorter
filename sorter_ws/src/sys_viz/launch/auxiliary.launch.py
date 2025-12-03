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
    )
    arduino_bridge_node = Node(
        package="teensy_pkg",
        executable="util_arduino_node",
        name="util_arduino_node",
    )
    transform_node = Node(
        package="transforms",
        executable="transform_node",
        name="transform_node",
    )
    item_detector_node = Node(
        package="perception",
        executable="item-detector",
        name="item_detector",
    )
    visualisation_node = Node(
        package="visualisation",
        executable="visualisation_node",
        name="visualisation_node",
    )


    return launch.LaunchDescription([
        arduino_bridge_node,
        item_detector_node,
        visualisation_node,
        transform_node,
        brain_node,
    ])