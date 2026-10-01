#!/usr/bin/env python3
"""Simulation bring-up for T3: bag playback + lio_node + the REAL localizer_node + the REAL
robot_pose consumer (`map_pose_publisher`).

Identical to sim_fastlio2.launch.py (which it includes) plus three extra processes:

    localizer_node        namespace /localizer  config_path:=<sim localizer config>
    map_pose_publisher    package robot_pose    (the T3 CONSUMER UNDER TEST, not a surrogate)
    static_transform_publisher  body -> base_link   (SIMULATION-ONLY identity)

The localizer consumes lio_node's /fastlio2/body_cloud + /fastlio2/lio_odom, aligns them
against the FROZEN prior PCD map (pcd_path) and broadcasts TF map -> odom.  It is a genuinely
separate consumer: it never sees the trajectory, the ground truth or the mapping odometry
labels, and it never updates the prior map.

`map_pose_publisher` is the actual consumer: it composes map <- odom (localizer broadcast) with
odom <- body (lio odometry TF) and body -> base_link (the static TF below) into /robot_pose_map.
`consumer_mode` selects its semantics with ONE argument:

    consumer_mode:=stamp     current_pose_mode=false predict_current_pose=false  (default)
    consumer_mode:=current   current_pose_mode=true  predict_current_pose=false
    consumer_mode:=predict   current_pose_mode=true  predict_current_pose=true

The publisher refuses `predict_current_pose:=true` without `current_pose_mode:=true` (it exits
at startup), so an unsupported combination cannot silently run.

The static TF is SIMULATION-ONLY and named-argument explicit: the synthetic ground truth places
the lidar at the entity's origin and lio_node publishes the odom child frame as `body`, so the
body -> base_link transform of the same physical entity is the identity BY CONSTRUCTION.  This is
NOT a hardware calibration (a deployment profile carries its own measured extrinsic) and must
never be copied into one.

Every node that reads the ROS clock runs with `use_sim_time=true`: the bag carries 2023-era
stamps and /clock is published by the player, so TF lookups and the localizer's validity ages are
in bag time.  lio_node is CLOCK-AGNOSTIC by implementation (it stamps odometry from the message
times and never calls the node clock - checked in src/sensing/fastlio2/src/lio_node.cpp), so the
include needs no clock parameter.

Playback ownership: by default this launch owns `ros2 bag play` (exactly as before) and the
playback ending shuts the launch down.  `play_bag:=false shutdown_gate:=<file>` instead starts
EVERY consumer/recorder-side node first and lets the caller own playback: the caller then runs
`ros2 bag play` only after its recorders reported ready (no fixed-delay race), and ends the run
by creating <file>.  `shutdown_gate` has a bounded wait so a caller that dies without signalling
cannot leave the launch alive for ever.

    ros2 launch sim_localization.launch.py bag_path:=/tmp/bag work_dir:=/tmp/sim \\
        localizer_config:=/tmp/sim/loc_sim.yaml prior_map:=/tmp/sim/frozen_map.pcd \\
        consumer_mode:=predict

    # caller-owned playback (run_localization_sim.sh): no playback here, ends when <file> appears
    ros2 launch sim_localization.launch.py bag_path:=/tmp/bag work_dir:=/tmp/sim \\
        localizer_config:=/tmp/sim/loc_sim.yaml prior_map:=/tmp/sim/frozen_map.pcd \\
        consumer_mode:=predict play_bag:=false shutdown_gate:=/tmp/sim/run_complete.gate
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SIM_LAUNCH = os.path.join(_THIS_DIR, "sim_fastlio2.launch.py")

# consumer_mode -> (current_pose_mode, predict_current_pose).  The publisher itself rejects
# predict-without-current, so this table is the single place that defines the three modes.
CONSUMER_MODES = {
    "stamp": (False, False),
    "current": (True, False),
    "predict": (True, True),
}

# Waits for the caller's completion file and exits, which shuts the launch down.  Needed because
# with the caller owning `ros2 bag play` there is no in-launch playback process left to trigger
# the shutdown.  The wait is BOUNDED (shutdown_gate_timeout_s), so a caller that dies without
# signalling cannot leave the launch alive: the gate times out and the launch still ends.
_GATE_WATCH = (
    "import os, sys, time\n"
    "gate, limit = sys.argv[1], float(sys.argv[2])\n"
    "end = time.monotonic() + limit\n"
    "while time.monotonic() < end:\n"
    "    if os.path.exists(gate):\n"
    "        print('[shutdown_gate] %s appeared; shutting the launch down' % gate, flush=True)\n"
    "        sys.exit(0)\n"
    "    time.sleep(0.2)\n"
    "print('[shutdown_gate] %s did not appear within %.1fs; shutting the launch down anyway'\n"
    "      % (gate, limit), flush=True)\n"
    "sys.exit(0)\n"
)


def consumer_flags(mode):
    """(current_pose_mode, predict_current_pose) for a mode name; unknown modes raise."""
    key = (mode or "").strip().lower()
    if key not in CONSUMER_MODES:
        raise RuntimeError("consumer_mode must be one of %s (got %r)"
                           % (sorted(CONSUMER_MODES), mode))
    return CONSUMER_MODES[key]


def check_playback(play_bag, gate):
    """Refuse a configuration that could never end - or that would end twice."""
    if not play_bag and not gate:
        raise RuntimeError("play_bag:=false needs shutdown_gate:=<file>: with the caller owning "
                           "`ros2 bag play` nothing else could ever end this launch")
    if play_bag and gate:
        raise RuntimeError("play_bag:=true and shutdown_gate:=<file> are mutually exclusive: the "
                           "in-launch playback already ends the launch")


def _compose(context, *args, **kwargs):
    mode = LaunchConfiguration("consumer_mode").perform(context)
    current_pose_mode, predict_current_pose = consumer_flags(mode)
    play_bag = LaunchConfiguration("play_bag").perform(context).lower() in ("1", "true", "yes")
    gate = LaunchConfiguration("shutdown_gate").perform(context)
    gate_timeout = LaunchConfiguration("shutdown_gate_timeout_s").perform(context)
    check_playback(play_bag, gate)

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_SIM_LAUNCH),
            launch_arguments={
                "bag_path": LaunchConfiguration("bag_path"),
                "work_dir": LaunchConfiguration("work_dir"),
                "config_file": LaunchConfiguration("config_file"),
                "extra_config": LaunchConfiguration("extra_config"),
                "rate": LaunchConfiguration("rate"),
                "start_delay": LaunchConfiguration("start_delay"),
                "play_bag": "true" if play_bag else "false",
                "rviz": "false",
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        ),
        # the REAL T3 consumer under test (never a surrogate composition)
        Node(
            package="robot_pose",
            executable="map_pose_publisher",
            name="map_pose_publisher",
            output="screen",
            parameters=[{
                "current_pose_mode": current_pose_mode,
                "predict_current_pose": predict_current_pose,
                "use_sim_time": True,
            }],
        ),
        # SIMULATION-ONLY identity TF of the same physical entity (named arguments)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sim_body_base_link_tf",
            output="screen",
            arguments=["--frame-id", "body", "--child-frame-id", "base_link",
                       "--x", "0.0", "--y", "0.0", "--z", "0.0",
                       "--qx", "0.0", "--qy", "0.0", "--qz", "0.0", "--qw", "1.0"],
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="localizer",
            executable="localizer_node",
            namespace="localizer",
            name="localizer_node",
            output="screen",
            parameters=[{
                "config_path": LaunchConfiguration("localizer_config"),
                "pcd_path": LaunchConfiguration("prior_map"),
                "use_sim_time": True,
            }],
        ),
    ]
    if gate:
        actions.append(ExecuteProcess(
            cmd=["python3", "-c", _GATE_WATCH, gate, gate_timeout],
            name="runner_completion_gate",
            output="screen",
            on_exit=Shutdown(reason="caller signalled run complete (or the gate timed out)"),
        ))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument("bag_path", default_value="", description="synthetic bag to play"),
        DeclareLaunchArgument("work_dir", default_value="/tmp/fastlio2_localization_sim"),
        DeclareLaunchArgument("config_file", default_value="",
                              description="effective lio_node config (written by sim_config.py)"),
        DeclareLaunchArgument("extra_config", default_value="",
                              description="optional ablation overlay merged last"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("start_delay", default_value="3.0"),
        DeclareLaunchArgument("play_bag", default_value="true",
                              description="false = the caller owns `ros2 bag play`; then "
                                          "shutdown_gate:=<file> is required"),
        DeclareLaunchArgument("shutdown_gate", default_value="",
                              description="file whose appearance ends the launch (caller-owned "
                                          "playback only)"),
        DeclareLaunchArgument("shutdown_gate_timeout_s", default_value="900.0",
                              description="upper bound on the shutdown_gate wait"),
        DeclareLaunchArgument("consumer_mode", default_value="stamp",
                              description="stamp | current | predict (map_pose_publisher mode)"),
        DeclareLaunchArgument("localizer_config", default_value="",
                              description="effective localizer config (sim_localizer_config.py)"),
        DeclareLaunchArgument("prior_map", default_value="",
                              description="frozen prior PCD map in the scene frame"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_compose)])
