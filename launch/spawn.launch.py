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

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = [
    DeclareLaunchArgument(
        "namespace", default_value="empty_namespace", description="entity namespace"
    ),
    DeclareLaunchArgument(
        "spawn_pose",
        default_value="[0.0, 0.0, 0.0, 0.0]",
        description="Spawn pose [x, y, z, yaw] (floats)",
    ),
]


def launch_setup(context, *args, **kwargs):
    spawn_pose = ParameterValue(LaunchConfiguration("spawn_pose"), value_type=List[float]).evaluate(
        context
    )
    if len(spawn_pose) != 4:
        raise ValueError(f"spawn_pose must be [x, y, z, yaw], got {spawn_pose}")

    spawn_entity = Node(
        package="ros_gz_sim",
        name="create",
        executable="create",
        arguments=[
            "-name",
            LaunchConfiguration("namespace"),
            "-x",
            str(spawn_pose[0]),
            "-y",
            str(spawn_pose[1]),
            "-z",
            str(spawn_pose[2]),
            "-Y",
            str(spawn_pose[3]),
            "-topic",
            "robot_description",
        ],
        output="screen",
    )
    return [spawn_entity]


def generate_launch_description():
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
