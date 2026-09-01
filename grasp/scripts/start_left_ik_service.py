#!/usr/bin/env python3
"""Run a namespaced, solver-only MoveIt service for the live left-arm cycle."""

from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

from franka_mobile_fr3_duo_moveit_config.description import get_robot_descriptions
from franka_mobile_fr3_duo_moveit_config.parameters import get_combined_parameters


def description():
    robot_description, robot_description_semantic = get_robot_descriptions(
        "mobile_fr3_duo_v0_2", "false"
    )
    parameters = get_combined_parameters(
        robot_description, robot_description_semantic, False
    )
    parameters.append(
        {
            "allow_trajectory_execution": False,
            "moveit_manage_controllers": False,
            "disable_capabilities": " ".join(
                [
                    "move_group/MoveGroupExecuteTrajectoryAction",
                    "move_group/MoveGroupExecuteService",
                    "move_group/MoveGroupMoveAction",
                ]
            ),
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
        }
    )
    return LaunchDescription(
        [
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                namespace="left_ik",
                name="move_group",
                output="screen",
                parameters=parameters,
                remappings=[
                    (
                        "joint_states",
                        "/left/franka_robot_state_broadcaster/measured_joint_states",
                    )
                ],
            )
        ]
    )


if __name__ == "__main__":
    service = LaunchService()
    service.include_launch_description(description())
    raise SystemExit(service.run())
