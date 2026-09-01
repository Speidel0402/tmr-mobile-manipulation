from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="config/left.yaml"),
        Node(
            package="rim_grasp_perception",
            executable="rim_grasp_node",
            name="left_rim_grasp",
            parameters=[{"config": LaunchConfiguration("config")}],
            output="screen",
        ),
    ])
