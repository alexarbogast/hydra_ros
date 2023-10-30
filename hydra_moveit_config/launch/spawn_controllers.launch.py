from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    GroupAction
)
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
#from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("hydra", package_name="hydra_moveit_config").to_moveit_configs()
    ld = LaunchDescription()
    robots = ["rob1", "rob2", "rob3"]

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("hydra_moveit_config"), "config", "ros2_controllers.yaml"]
    )

    for robot in robots:
        launch_ns_robot = GroupAction(
            actions = [
                PushRosNamespace(robot),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare("za_robot"),
                            "launch", "ros_controllers.launch.py",
                        ]),
                    ]),
                    launch_arguments={
                        "prefix": robot + "_",
                        "controller_config": robot_controllers, 
                    }.items()
                )
            ]
        )
        ld.add_action(launch_ns_robot)

    positioner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("hydra_bringup"),
                "launch", "hydra_positioner.launch.py",
            ])
        ]),
        launch_arguments={
            "namespace": "positioner"
        }.items()
    )
    ld.add_action(positioner_launch)

    return ld
