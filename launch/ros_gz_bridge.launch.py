from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import tempfile

ARGUMENTS = [
    DeclareLaunchArgument(
        "namespace", default_value="empty_namespace", description="Robot namespace"
    ),
    DeclareLaunchArgument("world", default_value="undefined", description="World name"),
    DeclareLaunchArgument("config_file", description="Path to the config file"),
]


def generate_bridge_params(namespace, world, config_file):

    # Read the template file
    with open(config_file) as f:
        content = f.read()

    # Replace placeholders
    content = content.replace("<robot_namespace>", namespace)
    content = content.replace("<world_namespace>", world)

    # Write to a temp file
    tmp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml")
    tmp_file.write(content.encode())
    tmp_file.close()
    return tmp_file.name


# This function is called at launch-time with the LaunchContext
def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    world = LaunchConfiguration("world").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)

    params_file = generate_bridge_params(namespace, world, config_file)

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={params_file}",
        ],
        output="screen",
    )

    return [ros_gz_bridge]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)

    # Use OpaqueFunction to delay evaluation until runtime
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
