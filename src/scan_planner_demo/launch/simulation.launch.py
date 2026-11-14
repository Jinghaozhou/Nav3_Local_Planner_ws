"""PCD-driven Go2 / Gazebo Fortress integration test for SCAN Planner."""

import os
import json
import math
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
    scan_config_arg = value("scan_config_file")
    scan_config = (Path(scan_config_arg).expanduser().resolve() if scan_config_arg
                   else share / "config" / "scan_demo.yaml")
    if not scan_config.is_file():
        raise ValueError(f"scan_config_file must point to an existing file: {scan_config}")
    path_z_offset = float(value("path_z_offset"))
    if not math.isfinite(path_z_offset):
        raise ValueError("path_z_offset must be finite")
    path_source = value("path_source")
    if path_source not in ("scenario", "pct"):
        raise ValueError("path_source must be 'scenario' or 'pct'")
    pcd = Path(value("pcd_map_file")).expanduser().resolve()
    world = Path(value("world_file")).expanduser().resolve()
    for name, path in (("pcd_map_file", pcd), ("world_file", world)):
        if not path.is_file():
            raise ValueError(f"{name} must point to an existing file: {path}")
    if path_source == "scenario":
        scenario = Path(value("scenario_config")).expanduser().resolve()
        if not scenario.is_file():
            raise ValueError(f"scenario_config must point to an existing file: {scenario}")
        path_node = Node(package="scan_planner_demo", executable="path_publisher.py",
                         namespace="scan_demo", output="screen",
                         parameters=[str(scenario), {"use_sim_time": True}])
    else:
        pct_path_topic = value("pct_path_topic")
        pct_path_z_offset = float(value("pct_path_z_offset"))
        if (not pct_path_topic.startswith("/") or pct_path_topic == "/scan_demo/global_path"
                or not math.isfinite(pct_path_z_offset)):
            raise ValueError("Invalid pct_path_topic or pct_path_z_offset")
        path_node = Node(package="scan_planner_demo", executable="pct_path_adapter.py",
                         namespace="scan_demo", output="screen", parameters=[{
                             "use_sim_time": True, "input_topic": pct_path_topic,
                             "source_frame": "map", "target_frame": "scan_demo_map",
                             "z_offset": pct_path_z_offset,
                         }])
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
    spawn_position = tuple(float(value(axis)) for axis in ("spawn_x", "spawn_y", "spawn_z"))
    if not all(math.isfinite(coordinate) for coordinate in spawn_position):
        raise ValueError("Go2 spawn coordinates must be finite")
    if not (bounds[0] <= spawn_position[0] < bounds[1] and
            bounds[2] <= spawn_position[1] < bounds[3]):
        raise ValueError("Go2 spawn XY is outside the converted PCD world bounds")

    gz_share = Path(get_package_share_directory("ros_gz_sim"))

    model = share / "description" / "xacro" / "robot.xacro"
    robot_description = Command(["xacro ", str(model), " use_gazebo:=true"])
    gait_controller = value("gait_controller")
    if gait_controller not in ("champ", "simple"):
        raise ValueError("gait_controller must be 'champ' or 'simple'")
    if gait_controller == "champ":
        gait = Node(package="scan_planner_demo", executable="go2_champ_gait",
                    namespace="scan_demo", output="screen",
                    parameters=[str(share / "config" / "gait_champ.yaml"),
                                {"use_sim_time": True, "robot_description": robot_description}])
    else:
        gait = Node(package="scan_planner_demo", executable="go2_cmd_vel_gait",
                    namespace="scan_demo", output="screen",
                    parameters=[str(share / "config" / "gait.yaml"), {"use_sim_time": True}])
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
            "path_z_offset": str(path_z_offset),
            "unknown_space_policy": "free_inside_bounds",
            "override_config": str(scan_config),
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
        path_node,
        gait,
        scan,
    ]
    if value("rviz").lower() == "true":
        rviz = Node(package="rviz2", executable="rviz2", output="screen",
                    arguments=["-d", str(share / "rviz" / "integration.rviz")],
                    parameters=[{"use_sim_time": True}])
        actions.append(RegisterEventHandler(OnProcessExit(target_action=state_broadcaster,
                                                         on_exit=[rviz])))
    return actions


def generate_launch_description():
    defaults = {
        "pcd_map_file": "", "world_file": "", "scenario_config": "",
        "scan_config_file": "",
        "path_source": "scenario", "pct_path_topic": "/pct/global_path",
        "pct_path_z_offset": "-0.10",
        "spawn_x": "0.0", "spawn_y": "0.0", "spawn_z": "0.45",
        "downsample_resolution": "0.1", "crop_half_xy": "4.5", "crop_half_z": "2.0",
        "path_height_mode": "reference", "path_z_offset": "0.0",
        "velocity_source": "odometry_twist",
        "rviz": "false", "headless": "false",
        "start_scan": "true",
        "gait_controller": "champ",
    }
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default)
                              for name, default in defaults.items()] + [OpaqueFunction(function=_setup)])
