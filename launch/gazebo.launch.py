import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    GroupAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Paths
    pkg_duatic_simulation = get_package_share_directory("duatic_simulation")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])

    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join(
            [
                os.path.join(pkg_duatic_simulation, "worlds"),
                os.path.join(pkg_duatic_simulation, "models"),
            ]
        ),
    )

    # Launch Gazebo headless or with GUI
    gz_args = [
        LaunchConfiguration("world"),
        ".sdf",
        " -r",
        " -v " + LaunchConfiguration("log_level").perform(context),
    ]

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

    # Compose launch description
    return [gz_resource_path, gazebo, clock_bridge]


def generate_launch_description():
    # Define LaunchDescription variable
    ld = LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="empty", description="Simulation World"),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                choices=["false", "true"],
                description="Run the simulation headless",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="2",
                description="Set the Gazebo log level",
            ),
        ]
    )

    # Add nodes to LaunchDescription
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
