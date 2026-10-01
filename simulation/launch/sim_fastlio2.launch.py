#!/usr/bin/env python3
"""FastLIO2 simulation bring-up: synthetic bag playback + lio_node (+ optional RViz).

    ros2 launch <path>/sim_fastlio2.launch.py                     # full 60 s run
    ros2 launch <path>/sim_fastlio2.launch.py rviz:=false
    ros2 launch <path>/sim_fastlio2.launch.py bag_path:=/tmp/bag duration_hint:=20

Why a generated config instead of passing the package yaml straight through:
the node reads every topic/frame/type setting from a single yaml file (``config_path``), so a
simulation needs ``lidar_type: pointcloud2`` (generic PointCloud2, not Livox CustomMsg),
the bag's topic names, a per-point time field and ``imu_acc_scale: 1.0`` (the synthetic IMU
already reports m/s^2).  Rather than duplicating config/lio.yaml -- which would silently go
stale as the C2.1/C2.2 work lands in the package config -- this launch file loads the package
config (source tree by default, install share as fallback), applies only those sim-relevant
overrides, writes the result to ``work_dir/lio_sim.yaml`` and passes it to the node.

``extra_config:=<yaml>`` is merged last, so an ablation (e.g. a config that flips the C2.1
normalisation or the C2.2 init mode) can be layered on without editing this file.  Keys the
node does not read are ignored by it, exactly as in a normal run.

``play_bag:=false`` starts the nodes WITHOUT playback, so a caller that owns ``ros2 bag play``
itself (run_localization_sim.sh) can first bring up every consumer and recorder, wait for the
real readiness handshake and only then start the bag - removing the fixed-delay race between
recorder start and first scan.  The default (``true``) is byte-identical to the previous
behaviour: playback in the launch, ``on_exit=Shutdown`` when it ends.
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            Shutdown)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))          # <repo>/simulation/launch -> <repo>
_SOURCE_CONFIG = os.path.join(_REPO_ROOT, "src", "sensing", "fastlio2", "config", "lio.yaml")
_DEFAULT_BAG = os.path.join(_REPO_ROOT, "simulation", "synthetic_data", "test_bag")

# Simulation overrides.  Values are in the exact units the node expects.
SIM_OVERRIDES = {
    "lidar_type": "pointcloud2",       # subscribe sensor_msgs/PointCloud2 on lidar_topic
    "lidar_topic": "/rslidar_points",
    "imu_topic": "/imu/data",
    "pcl2_time_field": "time",         # per-point sweep time, seconds since the first point
    "pcl2_time_scale": 1.0,
    "imu_acc_scale": 1.0,              # synthetic IMU is already m/s^2 (MID360 needs 10.0)
    "body_frame": "body",
    "world_frame": "odom",
    # SIMULATION ONLY: the synthetic cloud is rendered in the body frame, so the lidar->IMU
    # extrinsic is the identity BY CONSTRUCTION.  This is NOT a deployment calibration and
    # must never be copied into a hardware profile (the deployment profile carries its own,
    # measured extrinsic).
    "ext_il": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
}


def _resolve_config():
    """Package config: the source tree wins so freshly edited config is what runs."""
    if not os.path.exists(_SOURCE_CONFIG):
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("fastlio2"), "config", "lio.yaml")
    return _SOURCE_CONFIG


def _compose(context, *args, **kwargs):
    bag_path = LaunchConfiguration("bag_path").perform(context)
    work_dir = LaunchConfiguration("work_dir").perform(context)
    extra_config = LaunchConfiguration("extra_config").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    rate = LaunchConfiguration("rate").perform(context)
    delay = LaunchConfiguration("start_delay").perform(context)
    use_rviz = LaunchConfiguration("rviz").perform(context).lower() in ("1", "true", "yes")

    base = config_file or _resolve_config()
    with open(base, "r") as fh:
        config = yaml.safe_load(fh) or {}
    config.update(SIM_OVERRIDES)
    if extra_config:
        with open(extra_config, "r") as fh:
            override = yaml.safe_load(fh) or {}
        config.update(override)

    # the simulation is only meaningful if these hold; fail loudly instead of silently
    # measuring a mis-topic'd or mis-scaled run
    violated = [k for k, v in SIM_OVERRIDES.items() if config.get(k) != v]
    if violated:
        raise RuntimeError(
            "extra_config %r overrode simulation-critical keys %s (expected %s); "
            "ablation configs must not change the bag interface"
            % (extra_config, violated, {k: SIM_OVERRIDES[k] for k in violated}))

    os.makedirs(work_dir, exist_ok=True)
    effective = os.path.join(work_dir, "lio_sim.yaml")
    with open(effective, "w") as fh:
        yaml.safe_dump(config, fh, sort_keys=False)

    play_bag = LaunchConfiguration("play_bag").perform(context).lower() in ("1", "true", "yes")

    actions = []
    if play_bag:
        if not os.path.isdir(bag_path):
            raise RuntimeError(
                "bag not found: %s -- generate it first with "
                "python3 simulation/synthetic_data/generate_test_bag.py" % bag_path)
        actions.append(ExecuteProcess(
            cmd=["ros2", "bag", "play", bag_path, "--clock", "--rate", rate,
                 "--delay", delay, "--disable-keyboard-controls"],
            output="screen",
            name="bag_play",
            on_exit=Shutdown(reason="bag playback finished"),
        ))
    actions.append(
        Node(
            package="fastlio2",
            executable="lio_node",
            namespace="fastlio2",
            name="lio_node",
            output="screen",
            arguments=["--ros-args", "--log-level",
                       LaunchConfiguration("log_level").perform(context)],
            parameters=[{"config_path": effective}],
        ))
    if use_rviz:
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            namespace="fastlio2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(_THIS_DIR, "rviz_config.rviz")],
            # the bag carries 2023-era stamps and bag play publishes /clock, so RViz must read
            # simulated time or every TF/message lookup would be "into the past"
            parameters=[{"use_sim_time": True}],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bag_path", default_value=_DEFAULT_BAG,
                              description="synthetic rosbag2 directory to play"),
        DeclareLaunchArgument("work_dir", default_value="/tmp/fastlio2_sim",
                              description="where the effective sim config is written"),
        DeclareLaunchArgument("extra_config", default_value="",
                              description="optional yaml merged last (ablation config); "
                                          "simulation-critical keys are rejected if changed"),
        DeclareLaunchArgument("config_file", default_value="",
                              description="base profile yaml (default: the fastlio2 source "
                                          "config/lio.yaml; e.g. config/lio_orin_nx.yaml for the "
                                          "deployment profile)"),
        DeclareLaunchArgument("rate", default_value="1.0", description="bag playback rate"),
        DeclareLaunchArgument("start_delay", default_value="2.0",
                              description="delay before playback, so the node is subscribed"),
        DeclareLaunchArgument("play_bag", default_value="true",
                              description="false = start the nodes without playback; the caller "
                                          "then owns `ros2 bag play` (and its own readiness gate)"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        OpaqueFunction(function=_compose),
    ])
