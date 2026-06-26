import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_crazyflies_yaml = os.path.join(
        get_package_share_directory("crazyflie"),
        "config",
        "crazyflies.yaml",
    )
    default_config_yaml = os.path.join(
        get_package_share_directory("crazyswarm_application"),
        "launch",
        "config.yaml",
    )

    crazyflies_path = LaunchConfiguration("crazyflies_path")
    config_path = LaunchConfiguration("config_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "crazyflies_path",
                default_value=default_crazyflies_yaml,
                description="Path to crazyflie robots yaml",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value=default_config_yaml,
                description="Path to crazyswarm_application config yaml",
            ),
            Node(
                package="crazyswarm_application",
                executable="smaclike_aviary.py",
                name="smaclike_node",
                output="screen",
                parameters=[
                    {"crazyflies_path": crazyflies_path},
                    {"config_path": config_path},
                ],
            ),
        ]
    )
