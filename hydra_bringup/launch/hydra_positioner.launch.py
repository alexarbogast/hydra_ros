from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
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
    controller = PathJoinSubstitution(
        [FindPackageShare("hydra_bringup"), "config", "positioner_controllers.yaml"]
    )
        
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="positioner",
            description="The namespace for the control nodes",
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
            "controller_config",
            default_value=controller,
            description="Path to the configuration file for ros2_control"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Should mock (simulated) hardware be used?",
        )
    )
    namespace = LaunchConfiguration("namespace")
    controller = LaunchConfiguration("controller")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    controller_config = LaunchConfiguration("controller_config")

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("hydra_description"),
                "urdf",
                "positioner",
                "positioner.xacro",
                ]),
            " use_mock_hardware:=", use_mock_hardware,
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            controller_config,
        ],
        output="both",
        namespace=namespace
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", 
            "--controller-manager", "controller_manager",
        ],
        namespace=namespace
    )
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "--controller-manager", "controller_manager"],
        namespace=namespace
    )

    nodes_to_start = [
        control_node,
        joint_state_broadcaster_spawner,
        controller_spawner,
    ]
    return LaunchDescription(declared_arguments + nodes_to_start)
