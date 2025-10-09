from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    Command,
    FindExecutable,
)

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("hydra_bringup"), "config", "robot_controllers.yaml"]
    )

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
            "tool",
            default_value="typhoon_extruder",
            description="Tool: 'typhoon_extruder', or 'tool0'",
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

    arm_id = LaunchConfiguration("arm_id")
    tool = LaunchConfiguration("tool")
    controller = LaunchConfiguration("controller")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("hydra_description"),
                    "urdf",
                    "robots",
                    "za_tool.xacro",
                ]
            ),
            " use_mock_hardware:=",
            use_mock_hardware,
            " prefix:=",
            arm_id,
            TextSubstitution(text="_"),
            " tool:=",
            tool,
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            robot_controllers,
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "--controller-manager", "controller_manager"],
    )

    namespaced_group = GroupAction(
        actions=[
            PushRosNamespace(arm_id),
            control_node,
            joint_state_broadcaster_spawner,
            robot_controller_spawner,
        ]
    )

    return LaunchDescription(declared_arguments + [namespaced_group])
