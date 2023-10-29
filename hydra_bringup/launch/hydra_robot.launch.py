from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    arm_id = LaunchConfiguration("arm_id")
    controller = LaunchConfiguration("controller")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    rviz = LaunchConfiguration("rviz")

    # fmt: off
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("hydra_bringup"), "config", "robot_controllers.yaml"]
    )

    launch_ns_robot = GroupAction(
        actions=[
            PushRosNamespace(arm_id),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("za_robot"), 
                        "launch", "za_robot.launch.py",
                    ]),
                ]),
                launch_arguments={
                    "prefix": arm_id.perform(context) + "_",
                    "controller": controller,
                    "use_mock_hardware": use_mock_hardware,
                    "controller_config": robot_controllers,
                    "rviz": rviz
                }.items()
            ),
        ]
    )
    # fmt: on

    nodes_to_start = [launch_ns_robot]
    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_id",
            description="Name (prefix) of the robot to launch",
            choices=["rob1", "rob2", "rob3"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller",
            default_value="joint_trajectory_controller",
            description="Which controller should be started?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Should mock (simulated) hardware be used?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Should RViz be launched",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
