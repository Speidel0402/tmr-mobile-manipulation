"""Launch MoveIt's left-arm IK service without spawning or commanding controllers."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path

from franka_mobile_fr3_duo_moveit_config.description import get_robot_descriptions
from franka_mobile_fr3_duo_moveit_config.parameters import get_combined_parameters


def generate_launch_description():
    robot_description, robot_description_semantic = get_robot_descriptions(
        "mobile_fr3_duo_v0_2", "false"
    )
    move_group_parameters = get_combined_parameters(
        robot_description, robot_description_semantic, False
    )
    # These overrides make the process a solver/planning-scene provider only.
    # It must never execute trajectories or manage ros2_control controllers.
    move_group_parameters.append({
        "allow_trajectory_execution": False,
        "moveit_manage_controllers": False,
        "disable_capabilities": " ".join([
            "move_group/MoveGroupExecuteTrajectoryAction",
            "move_group/MoveGroupExecuteService",
            "move_group/MoveGroupMoveAction",
        ]),
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    })

    config_path = Path(
        get_package_share_directory("rim_grasp_perception")
    ) / "config" / "left_ik.yaml"

    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            namespace="left_ik",
            name="move_group",
            output="screen",
            parameters=move_group_parameters,
            remappings=[
                ("joint_states", "/left_ik/joint_states"),
            ],
        ),
        Node(
            package="rim_grasp_perception",
            executable="left_ik_client",
            name="left_ik_client",
            output="screen",
            parameters=[str(config_path)],
        ),
    ])
