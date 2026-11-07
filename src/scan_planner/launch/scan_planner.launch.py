"""Independent SCAN module; starts no sensors, localization, global planner or robot model."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    config = Path(get_package_share_directory("scan_planner")) / "config"
    boolean = lambda key: value(key).lower() == "true"
    common = {
        "use_sim_time": boolean("use_sim_time"),
        "global_frame": value("global_frame"),
        "planning_frame": value("planning_frame"),
        "robot_frame": value("robot_frame"),
        "control_frame": value("control_frame"),
        "velocity_source": value("velocity_source"),
        "path_height_mode": value("path_height_mode"),
        "path_z_offset": float(value("path_z_offset")),
        "unknown_space_policy": value("unknown_space_policy"),
        "require_localization_permission": boolean("require_localization_permission"),
    }
    nodes = []
    for executable, name, specific in (
        ("planner_node", "scan_planner", "planner.yaml"),
        ("controller_node", "scan_controller", "controller.yaml"),
    ):
        nodes.append(Node(
            package="scan_planner", executable=executable, name=name,
            namespace=value("namespace"), output="screen",
            parameters=[value("integration_config"), str(config / specific)]
            + ([value("override_config")] if value("override_config") else []) + [common],
            remappings=[
                ("odom", value("odom_topic")), ("local_map", value("local_map_topic")),
                ("global_path", value("path_topic")), ("cmd_vel", value("cmd_vel_topic")),
                ("localization_permission", value("localization_permission_topic")),
            ],
        ))
    return nodes


def generate_launch_description():
    config = Path(get_package_share_directory("scan_planner")) / "config"
    arguments = {
        "namespace": "", "use_sim_time": "false",
        "integration_config": str(config / "integration.yaml"), "override_config": "",
        "odom_topic": "/fast_lio/odom", "local_map_topic": "/fast_lio/local_map",
        "path_topic": "/pct/global_path", "cmd_vel_topic": "/cmd_vel",
        "global_frame": "map", "planning_frame": "odom", "robot_frame": "base_link",
        "control_frame": "base_link", "velocity_source": "pose_difference",
        "path_height_mode": "reference", "path_z_offset": "0.0",
        "unknown_space_policy": "occupied", "require_localization_permission": "false",
        "localization_permission_topic": "/scan_planner/localization_permission",
    }
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in arguments.items()] + [OpaqueFunction(function=_setup)])
