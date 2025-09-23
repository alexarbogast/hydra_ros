import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


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
    # Merge joint states topics and publish robot state
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "source_list": [
                    "rob1/joint_states",
                    "rob2/joint_states",
                    "rob3/joint_states",
                    "positioner/joint_states",
                ],
                "rate": 50.0,
            }
        ],
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

    # Launch the move group node
    moveit_controllers = os.path.join(
        get_package_share_directory("hydra_bringup"),
        "config",
        "moveit_controllers.yaml",
    )
    moveit_config = (
        MoveItConfigsBuilder("hydra", package_name="hydra_moveit_config")
        .trajectory_execution(file_path=moveit_controllers)
        .to_moveit_configs()
    )
    move_group = generate_move_group_launch(moveit_config)

    # Launch the rviz visualization
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
        joint_state_publisher_node,
        robot_state_publisher,
        move_group,
        visualization_launch,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
