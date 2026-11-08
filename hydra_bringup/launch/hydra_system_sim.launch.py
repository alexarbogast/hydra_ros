from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Should RViz be launched",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool",
            default_value="typhoon_extruder",
            description="Tool: 'typhoon_extruder'  or 'tool0'",
        )
    )
    rviz = LaunchConfiguration("rviz")
    tool = LaunchConfiguration("tool")

    # fmt: off
    robots_launch = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("hydra_bringup"),
                    "launch",
                    "hydra_robot.launch.py",
                ])
            ]),
            launch_arguments={
                "arm_id": arm_id,
                "rviz": "false",
            }.items(),
        )
        for arm_id in {"rob1", "rob2", "rob3"}
    ]

    positioner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("hydra_bringup"),
                "launch", "hydra_positioner.launch.py",
            ])
        ]),
    )

    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("hydra_bringup"),
                "launch", "hydra_visualization.launch.py",
            ])
        ]),
        launch_arguments={
            "tool": tool
        }.items(),
        condition=IfCondition(rviz)
    )
    # fmt: on

    nodes_to_start = [
        visualization_launch,
        *robots_launch,
        positioner_launch,
    ]
    return LaunchDescription(declared_arguments + nodes_to_start)
