from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import tempfile
import xml.etree.ElementTree as ET

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
    """
    Extract robot name from URDF file by parsing the <robot name="..."> tag.
    Handles both .urdf and .xacro files.

    Args:
        urdf_file_path: Path to the URDF or URDF.xacro file

    Returns:
        Robot name as string, or None if not found or file doesn't exist
    """
    try:
        # Check if it's a xacro file
        if urdf_file_path.endswith(".xacro"):
            import xacro

            # Process xacro to get the URDF XML (returns DOM Document)
            doc = xacro.parse(open(urdf_file_path))
            xacro.process_doc(doc)
            # DOM API uses documentElement instead of getroot()
            root = doc.documentElement
        else:
            # Parse as regular URDF
            tree = ET.parse(urdf_file_path)
            root = tree.getroot()

        # The root element should be <robot name="...">
        if root.tagName == "robot" and root.hasAttribute("name"):
            return root.getAttribute("name")

        return None
    except Exception as e:
        print(f"[ros_gz_bridge] Warning: Could not extract robot name from URDF: {e}")
        return None


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
    urdf_file = LaunchConfiguration("urdf_file").perform(context)

    # Determine robot model name with 3-tier priority:
    # 1. First priority: namespace (if set and not default "empty_namespace")
    # 2. Second priority: extract from URDF file
    robot_name = None

    if namespace and namespace != "empty_namespace" and namespace != "":
        robot_name = namespace
    elif urdf_file:
        robot_name = extract_robot_name_from_urdf(urdf_file)
        if robot_name:
            print(f"[ros_gz_bridge] Extracted robot name from URDF: {robot_name}")

    # Final fallback if everything failed
    if not robot_name:
        robot_name = "default_robot"
        print(f"[ros_gz_bridge] Warning: Using fallback robot name: {robot_name}")

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
