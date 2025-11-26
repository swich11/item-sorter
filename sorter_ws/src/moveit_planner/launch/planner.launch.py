import launch

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=192.168.0.100",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
           "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur5e",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    return robot_description


def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    return robot_description_semantic


def generate_launch_description():
    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    planner_node = Node(
        package="moveit_planner",
        executable="planner",
        name="planner",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )
    arduino_bridge_node = Node(
        package="teensy_pkg",
        executable="util_arduino_node",
        name="util_arduino_node",
        output="screen",
    )

    # # Generate launch description with multiple components
    # container = ComposableNodeContainer(
    #     name="hybrid_planning_container",
    #     namespace="/",
    #     package="rclcpp_components",
    #     executable="component_container",
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package="moveit_hybrid_planning",
    #             plugin="moveit::hybrid_planning::GlobalPlannerComponent",
    #             name="global_planner",
    #             parameters=[
    #                 global_planner_param,
    #                 robot_description,
    #                 robot_description_semantic,
    #                 kinematics_yaml,
    #                 ompl_planning_pipeline_config,
    #             ],
    #         ),
    #         ComposableNode(
    #             package="moveit_hybrid_planning",
    #             plugin="moveit::hybrid_planning::LocalPlannerComponent",
    #             name="local_planner",
    #             parameters=[
    #                 local_planner_param,
    #                 robot_description,
    #                 robot_description_semantic,
    #                 kinematics_yaml,
    #             ],
    #         ),
    #         ComposableNode(
    #             package="moveit_hybrid_planning",
    #             plugin="moveit::hybrid_planning::HybridPlanningManager",
    #             name="hybrid_planning_manager",
    #             parameters=[hybrid_planning_manager_param],
    #         ),
    #     ],
    #     output="screen",
    # )

    return launch.LaunchDescription([planner_node, arduino_bridge_node])