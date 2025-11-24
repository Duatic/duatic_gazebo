from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        "namespace", default_value="empty_namespace", description="entity namespace"
    ),
    DeclareLaunchArgument("initial_pose_x", default_value="0.0", description="x position"),
    DeclareLaunchArgument("initial_pose_y", default_value="0.0", description="y position"),
    DeclareLaunchArgument("initial_pose_z", default_value="0.0", description="z position"),
    DeclareLaunchArgument("initial_pose_yaw", default_value="0.0", description="yaw rotation"),
]


def generate_launch_description():
    spawn_entity = Node(
        package="ros_gz_sim",
        name="create",
        executable="create",
        arguments=[
            "-name",
            LaunchConfiguration("namespace"),
            "-x",
            LaunchConfiguration("initial_pose_x"),
            "-y",
            LaunchConfiguration("initial_pose_y"),
            "-z",
            LaunchConfiguration("initial_pose_z"),
            "-Y",
            LaunchConfiguration("initial_pose_yaw"),
            "-topic",
            "robot_description",
        ],
        output="screen",
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_entity)
    return ld
