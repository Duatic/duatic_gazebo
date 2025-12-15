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
    DeclareLaunchArgument(
        "robot_name", 
        default_value="", 
        description="Robot model name in Gazebo (e.g., 'alpha', 'torso', 'platform'). If empty, uses namespace or defaults to 'alpha'."
    ),
]


def generate_bridge_params(robot_model_name, world, config_file):
    """
    Generate Gazebo bridge parameters by replacing placeholders in config file.
    
    Args:
        robot_model_name: The model name in Gazebo (e.g., 'alpha', 'torso', 'platform'). 
                          If empty, uses 'default_robot' as fallback since Gazebo requires a model name.
        world: The Gazebo world name
        config_file: Path to the YAML config template with placeholders
    """
    # Read the template file
    with open(config_file) as f:
        content = f.read()

    # Replace world namespace
    content = content.replace("<world_namespace>", world)
    
    # Replace robot namespace - use fallback if empty since Gazebo ALWAYS requires /model/<name>
    model_name = robot_model_name if robot_model_name else "default_robot"
    content = content.replace("<robot_namespace>", model_name)

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
    robot_name_param = LaunchConfiguration("robot_name").perform(context)

    # Determine robot model name:
    # 1. First priority: explicit robot_name parameter
    # 2. Second priority: namespace (if set)
    # 3. If both empty: pass empty string to remove /model/<robot_namespace> from topics
    robot_name = robot_name_param if robot_name_param else namespace
    
    params_file = generate_bridge_params(robot_name, world, config_file)

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
