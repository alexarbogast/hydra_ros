from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="The configuration file to use for RViz",
        )
    )
    rviz = LaunchConfiguration("rviz")

    # fmt: off
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                        FindPackageShare("hydra_moveit_config"),
                        "launch",
                        "move_group.launch.py",
                ]),
        ]),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("hydra_moveit_config"),
                "launch",
                "rsp.launch.py"
            ]),
        ]),
    )

    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                        FindPackageShare("hydra_moveit_config"),
                        "launch",
                        "moveit_rviz.launch.py",
                ]),
        ]),
        condition=IfCondition(rviz),
    )
    # fmt: on

    nodes_to_start = [
        move_group,
        robot_state_publisher,
        visualization_launch
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)