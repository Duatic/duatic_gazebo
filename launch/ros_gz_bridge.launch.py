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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import tempfile
import xml.etree.ElementTree as ET
import yaml

ARGUMENTS = [
    DeclareLaunchArgument(
        "namespace", default_value="empty_namespace", description="Robot namespace"
    ),
    DeclareLaunchArgument("world", default_value="undefined", description="World name"),
    DeclareLaunchArgument("config_file", description="Path to the config file"),
    DeclareLaunchArgument(
        "urdf_file",
        default_value="",
        description="Path to URDF file for extracting robot name. Optional - only needed if namespace and robot_name are both empty.",
    ),
]


def extract_robot_name_from_urdf(urdf_file_path):
    """Extract robot name from URDF file by parsing the <robot name="..."> tag."""
    try:
        if urdf_file_path.endswith(".xacro"):
            import xacro

            doc = xacro.parse(open(urdf_file_path))
            xacro.process_doc(doc)
            root = doc.documentElement
        else:
            tree = ET.parse(urdf_file_path)
            root = tree.getroot()

        if root.tagName == "robot" and root.hasAttribute("name"):
            return root.getAttribute("name")

        return None
    except Exception as e:
        print(f"[ros_gz_bridge] Warning: Could not extract robot name from URDF: {e}")
        return None


def split_bridge_params(robot_model_name, world, config_file):
    """Reads the config file, replaces placeholders, and splits the configuration."""
    with open(config_file) as f:
        content = f.read()

    # Replace placeholders
    content = content.replace("<world_namespace>", world)
    model_name = robot_model_name if robot_model_name else "default_robot"
    content = content.replace("<robot_namespace>", model_name)

    # Parse the YAML configuration
    config_data = yaml.safe_load(content)

    standard_configs = []
    image_topics = []
    image_remappings = []

    # Common image_transport suffixes that get automatically appended
    image_transport_suffixes = [
        "",  # The base raw topic
        "/compressed",
        "/compressedDepth",
        "/theora",
        "/zstd",
    ]

    if config_data:
        for entry in config_data:
            if entry.get("ros_type_name") == "sensor_msgs/msg/ImageCompressed":
                gz_topic = entry.get("gz_topic_name", "").strip()
                ros_topic = entry.get("ros_topic_name", "").strip()

                # Force absolute path
                if not ros_topic.startswith("/"):
                    ros_topic = "/" + ros_topic

                image_topics.append(gz_topic)

                # Generate remapping rules for the base topic AND all compression plugins
                if gz_topic != ros_topic:
                    for suffix in image_transport_suffixes:
                        gz_subtopic = f"{gz_topic}{suffix}"
                        ros_subtopic = f"{ros_topic}{suffix}"
                        image_remappings.append((gz_subtopic, ros_subtopic))
            else:
                standard_configs.append(entry)

    standard_yaml_path = None
    if standard_configs:
        tmp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml", mode="w")
        yaml.dump(standard_configs, tmp_file)
        tmp_file.close()
        standard_yaml_path = tmp_file.name

    return standard_yaml_path, image_topics, image_remappings


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    world = LaunchConfiguration("world").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    urdf_file = LaunchConfiguration("urdf_file").perform(context)

    # Determine robot model name
    robot_name = None
    if namespace and namespace not in ["empty_namespace", ""]:
        robot_name = namespace
    elif urdf_file:
        robot_name = extract_robot_name_from_urdf(urdf_file)

    if not robot_name:
        robot_name = "default_robot"

    # Process and split the parameters
    params_file, image_topics, image_remappings = split_bridge_params(
        robot_name, world, config_file
    )

    nodes_to_launch = []

    # 1. Launch Standard Parameter Bridge
    if params_file:
        nodes_to_launch.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "--ros-args",
                    "-p",
                    f"config_file:={params_file}",
                ],
                output="screen",
            )
        )

    # 2. Launch Image Bridge
    if image_topics:
        nodes_to_launch.append(
            Node(
                package="ros_gz_image",
                executable="image_bridge",
                arguments=image_topics,
                remappings=image_remappings,
                output="screen",
            )
        )

    return nodes_to_launch


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
