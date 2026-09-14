# Copyright 2026 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
    pkg_duatic_gazebo = get_package_share_directory("duatic_gazebo")

    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])

    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            PathJoinSubstitution([pkg_duatic_gazebo, "worlds"]),  # world models within this repo
            ":",
            PathJoinSubstitution([pkg_duatic_gazebo, "models"]),  # object models within this repo
            ":",
            LaunchConfiguration("gz_models_path"),  # additional search paths provided by argument
        ],
    )

    # Launch Gazebo headless or with GUI. The world starts PAUSED (no -r):
    # gravity acting before ros2_control claims the joints lets the arms fall
    # limp and drag the hip over on slow starts. unpause_sim.py resumes
    # physics once the hardware components in wait_hardware are active (or
    # immediately if none are given).
    gz_args = [LaunchConfiguration("world"), ".sdf", " -v", LaunchConfiguration("log_level")]

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

    # Unpause once the requested controllers are active (see gz_args comment)
    unpause = Node(
        package="duatic_gazebo",
        executable="unpause_sim.py",
        name="unpause_sim",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "world": LaunchConfiguration("world"),
                "wait_hardware": LaunchConfiguration("wait_hardware"),
                "timeout": 60.0,
                # Must keep running while the paused world holds the clock still.
                "use_sim_time": False,
            }
        ],
    )

    return [gz_resource_path, gazebo, clock_bridge, unpause]


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
            "gz_models_path",
            default_value="",
            description="A ':'-separated list of Gazebo resource search paths",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="1",
            description="Gazebo log level(debug:4, info:3, warn:2, error:1, fatal:0)",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description=(
                "Namespace the robot's controller_manager lives in. unpause_sim "
                "looks for its services there."
            ),
        ),
        DeclareLaunchArgument(
            "wait_hardware",
            default_value="",
            description=(
                "Comma-separated ros2_control hardware component names that must be "
                "active before the paused simulation is resumed. Empty = unpause "
                "immediately."
            ),
        ),
    ]

    # Add nodes to LaunchDescription
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
