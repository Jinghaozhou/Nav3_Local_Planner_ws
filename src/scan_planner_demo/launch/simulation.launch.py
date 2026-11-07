"""PCD-driven Go2 / Gazebo Fortress integration test for SCAN Planner."""

import os
import json
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    share = Path(get_package_share_directory("scan_planner_demo"))
    planner_share = Path(get_package_share_directory("scan_planner"))
    pcd = Path(value("pcd_map_file")).expanduser().resolve()
    world = Path(value("world_file")).expanduser().resolve()
    scenario = Path(value("scenario_config")).expanduser().resolve()
    for name, path in (("pcd_map_file", pcd), ("world_file", world), ("scenario_config", scenario)):
        if not path.is_file():
            raise ValueError(f"{name} must point to an existing file: {path}")
    manifest = world.with_suffix(".json")
    if not manifest.is_file():
        raise ValueError(f"Missing PCD/world metadata; run pcd_to_sdf.py: {manifest}")
    metadata = json.loads(manifest.read_text(encoding="utf-8"))
    source = pcd.stat()
    mesh = Path(metadata["mesh_file"])
    if (metadata["pcd_file"] != str(pcd) or metadata["pcd_size"] != source.st_size or
            metadata["pcd_mtime_ns"] != source.st_mtime_ns or not mesh.is_file() or
            metadata["mesh_size"] != mesh.stat().st_size):
        raise ValueError("PCD/world metadata differ; regenerate SDF from the selected PCD")
    mesh_uri = ET.parse(world).findtext(".//model[@name='pcd_environment']/link/collision/geometry/mesh/uri")
    if mesh_uri != mesh.resolve().as_uri():
        raise ValueError("Gazebo collision mesh does not match PCD/world metadata")
    bounds = metadata["bounds"]
    if len(bounds) != 6 or not all(bounds[i] < bounds[i + 1] for i in (0, 2, 4)):
        raise ValueError("Invalid PCD/world bounds in conversion metadata")

    gz_share = Path(get_package_share_directory("ros_gz_sim"))

    model = share / "description" / "xacro" / "robot.xacro"
    robot_description = Command(["xacro ", str(model), " use_gazebo:=true"])
    gz_args = f"{'-s ' if value('headless').lower() == 'true' else ''}-r -v 3 {world}"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items())
    scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(planner_share / "launch" / "scan_planner.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_scan")),
        launch_arguments={
            "namespace": "scan_demo", "use_sim_time": "true",
            "odom_topic": "/scan_demo/odom", "local_map_topic": "/scan_demo/local_map",
            "path_topic": "/scan_demo/global_path", "cmd_vel_topic": "/scan_demo/cmd_vel",
            "global_frame": "scan_demo_map", "planning_frame": "scan_demo_odom",
            "robot_frame": "base", "control_frame": "base",
            "velocity_source": value("velocity_source"), "path_height_mode": value("path_height_mode"),
            "unknown_space_policy": "free_inside_bounds",
            "override_config": str(share / "config" / "scan_demo.yaml"),
        }.items())
    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "go2",
                            "-x", value("spawn_x"), "-y", value("spawn_y"),
                            "-z", value("spawn_z")])
    state_broadcaster = Node(package="controller_manager", executable="spawner",
                             output="screen", arguments=["joint_state_broadcaster",
                                                          "--controller-manager", "/controller_manager"])
    trajectory_controller = Node(package="controller_manager", executable="spawner",
                                 output="screen", arguments=["joint_trajectory_controller",
                                                              "--controller-manager", "/controller_manager"])
    actions = [
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH",
                               [str(share.parent), os.pathsep,
                                EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")]),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                               [str(share.parent), os.pathsep,
                                EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")]),
        gazebo,
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen", parameters=[{"robot_description": robot_description, "use_sim_time": True}]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": str(share / "config" / "gazebo_bridge.yaml")}]),
        Node(package="tf2_ros", executable="static_transform_publisher", output="screen",
             arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0",
                        "--pitch", "0", "--yaw", "0", "--frame-id", "scan_demo_map",
                        "--child-frame-id", "scan_demo_odom"]),
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[state_broadcaster, trajectory_controller])),
        Node(package="scan_planner_demo", executable="pcd_local_map_node", namespace="scan_demo",
             output="screen", parameters=[{
                 "use_sim_time": True, "pcd_file": str(pcd), "frame_id": "scan_demo_odom",
                 "downsample_resolution": float(value("downsample_resolution")),
                 "crop_half_xy": float(value("crop_half_xy")),
                 "crop_half_z": float(value("crop_half_z")),
                 "world_bounds": bounds,
             }], remappings=[("odom", "/scan_demo/odom")]),
        Node(package="scan_planner_demo", executable="path_publisher.py", namespace="scan_demo",
             output="screen", parameters=[str(scenario), {"use_sim_time": True}]),
        Node(package="scan_planner_demo", executable="go2_cmd_vel_gait", namespace="scan_demo",
             output="screen", parameters=[str(share / "config" / "gait.yaml"), {"use_sim_time": True}]),
        scan,
    ]
    if value("rviz").lower() == "true":
        actions.append(Node(package="rviz2", executable="rviz2", output="screen",
                            arguments=["-d", str(share / "rviz" / "integration.rviz")],
                            parameters=[{"use_sim_time": True}]))
    return actions


def generate_launch_description():
    defaults = {
        "pcd_map_file": "", "world_file": "", "scenario_config": "",
        "spawn_x": "0.0", "spawn_y": "0.0", "spawn_z": "0.45",
        "downsample_resolution": "0.1", "crop_half_xy": "4.5", "crop_half_z": "2.0",
        "path_height_mode": "reference", "velocity_source": "odometry_twist",
        "rviz": "false", "headless": "false",
        "start_scan": "true",
    }
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default)
                              for name, default in defaults.items()] + [OpaqueFunction(function=_setup)])
