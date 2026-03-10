import os
# from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)

from launch.substitutions import (
    PathJoinSubstitution,
    TextSubstitution,
    LaunchConfiguration,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            name="config_filepath",
            default_value=[
                TextSubstitution(
                    text=os.path.join(
                        get_package_share_directory("uwtec_navigation"), "config", ""
                    ),
                ),
                "micro",
                TextSubstitution(text=".config.yaml"),
            ],
        ),
    )
    config_filepath = LaunchConfiguration("config_filepath")

    # define nodes
    teleop_twist_joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("teleop_twist_joy"),
                    "launch",
                    "teleop-launch.py",
                ]
            )
        ),
        launch_arguments={
            "joy_config": "micro",
            "config_filepath": config_filepath,
            "joy_vel": "cmd_vel_joy",
            # "joy_vel": "/diff_drive_base_controller/cmd_vel",
            "publish_stamped_twist": "true",
        }.items(),
    )

    twist_mux_config_path = PathJoinSubstitution(
        [FindPackageShare("uwtec_navigation"), "config", "twist_mux.yaml"]
    )
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_config_path],
        # remappings=[("cmd_vel_out", "/diff_drive_base_controller/cmd_vel")],
        # remappings=[("cmd_vel_out", "/cmd_vel_mux")],
        remappings=[("cmd_vel_out", "/cmd_vel")],
    )

    # uros_cmd_vel_converter_node = Node(
    #     package="uwtec_navigation",
    #     executable="uros_cmd_vel_converter",
    #     name="uros_cmd_vel_converter_node",
    #     output="screen",
    #     # arguments=["--debug"],
    # )
    cmd_vel_joy_node = Node(
        package="uwtec_navigation",
        executable="cmd_vel_joy",
        # name="cmd_vel_joy_node",
        output="screen",
        # arguments=["--debug"],
    )

    nodes = [
        teleop_twist_joy_node,
        twist_mux_node,
        # uros_cmd_vel_converter_node,
        cmd_vel_joy_node,
    ]

    # create launch description and populate
    return LaunchDescription(declared_arguments + nodes)
