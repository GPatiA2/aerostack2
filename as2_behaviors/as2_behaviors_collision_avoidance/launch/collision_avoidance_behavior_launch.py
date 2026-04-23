from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from as2_core.launch_plugin_utils import get_available_plugins


def generate_launch_description():
    pkg = "as2_behaviors_collision_avoidance"

    config_file = PathJoinSubstitution([
        FindPackageShare(pkg), "config", "config_default.yaml"
    ])

    available_plugins = get_available_plugins(pkg, "collision_avoidance")
    default_plugin = available_plugins[0] if available_plugins else "pairwise_path_lock_plugin::Plugin"

    return LaunchDescription([
        DeclareLaunchArgument("namespace",        default_value=""),
        DeclareLaunchArgument("use_sim_time",     default_value="false"),
        DeclareLaunchArgument("log_level",        default_value="info"),
        DeclareLaunchArgument("config_file",      default_value=config_file),
        DeclareLaunchArgument("plugin_name",
            default_value=default_plugin,
            description=f"Plugin to load. Available: {available_plugins if available_plugins else ['pairwise_path_lock_plugin::Plugin']}"),

        Node(
            package=pkg,
            executable="collision_avoidance_behavior_node",
            namespace=LaunchConfiguration("namespace"),
            output="screen",
            arguments=["--ros-args", "--log-level",
                       LaunchConfiguration("log_level")],
            parameters=[
                LaunchConfiguration("config_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"plugin_name":  LaunchConfiguration("plugin_name")},
            ],
        ),
    ])
