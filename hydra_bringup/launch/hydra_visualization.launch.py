from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("hydra_bringup"), "rviz", "hydra.rviz"]
    )

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=default_rviz_config,
            description="The configuration file to use for RViz",
        )
    )
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("hydra_description"), "urdf", "hydra.xacro"]
            ),
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str),
             "publish_frequency": 50.0}
        ],
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {"source_list": ["rob1/joint_states",
                             "rob2/joint_states"],
             "rate": 50.0}
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ]
    return LaunchDescription(declared_arguments + nodes_to_start)
