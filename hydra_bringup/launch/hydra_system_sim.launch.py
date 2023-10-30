from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
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
    rviz = LaunchConfiguration("rviz")
    
    # fmt: off
    rob1_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("hydra_bringup"),
                "launch", "hydra_robot.launch.py",
            ])
        ]),
        launch_arguments={
            "arm_id": "rob1",
            "rviz": "false",
        }.items(),
    )
    rob2_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("hydra_bringup"),
                "launch", "hydra_robot.launch.py",
            ])
        ]),
        launch_arguments={
            "arm_id": "rob2",
            "rviz": "false",
        }.items(),
    )
    rob3_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("hydra_bringup"),
                "launch", "hydra_robot.launch.py",
            ])
        ]),
        launch_arguments={
            "arm_id": "rob3",
            "rviz": "false",
        }.items(),
    )
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
        condition=IfCondition(rviz)
    )
    # fmt: on

    nodes_to_start = [
        visualization_launch,
        rob1_robot_launch,
        rob2_robot_launch,
        rob3_robot_launch,
        positioner_launch,
    ]
    return LaunchDescription(declared_arguments + nodes_to_start)
