from moveit_configs_utils import MoveItConfigsBuilder
#from moveit_configs_utils.launches import generate_rsp_launch

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("hydra", package_name="hydra_moveit_config").to_moveit_configs()
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="50.0"))

    freq = LaunchConfiguration("publish_frequency")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        respawn=True,
        parameters=[moveit_config.robot_description,
                    {
                        "publish_frequency": freq
                    },
        ],
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        respawn=True,
        parameters=[
            {"source_list": ["rob1/joint_states",
                             "rob2/joint_states"],
             "rate": freq}
        ],
    )
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)

    return ld
