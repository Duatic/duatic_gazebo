import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Packages Directories
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])

    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            LaunchConfiguration("gz_worlds_path"),
            ":",
            LaunchConfiguration("gz_models_path"),
        ],
    )

    # Launch Gazebo headless or with GUI
    gz_args = [LaunchConfiguration("world"), ".sdf", " -r", " -v", LaunchConfiguration("log_level")]

    gazebo = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gz_sim_launch]),
                launch_arguments=[("gz_args", gz_args)],
                condition=UnlessCondition(LaunchConfiguration("headless")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gz_sim_launch]),
                launch_arguments=[("gz_args", gz_args + [" -s"])],
                condition=IfCondition(LaunchConfiguration("headless")),
            ),
        ]
    )

    # Clock bridge node
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    return [gz_resource_path, gazebo, clock_bridge]


def generate_launch_description():
    # Declare Launch Arguments
    declared_arguments = [
        DeclareLaunchArgument("world", default_value="empty", description="Simulation World"),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            choices=["false", "true"],
            description="Run the simulation headless",
        ),
        DeclareLaunchArgument(
            "gz_worlds_path",
            default_value=os.path.join(get_package_share_directory("duatic_gazebo"), "worlds"),
            description="Path to the Gazebo worlds directory",
        ),
        DeclareLaunchArgument(
            "gz_models_path",
            default_value=os.path.join(get_package_share_directory("duatic_gazebo"), "models"),
            description="Path to the Gazebo models directory",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="1",
            description="Gazebo log level(debug:4, info:3, warn:2, error:1, fatal:0)",
        ),
    ]

    # Add nodes to LaunchDescription
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
