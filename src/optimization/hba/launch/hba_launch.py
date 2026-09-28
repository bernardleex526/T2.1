import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_cfg = PathJoinSubstitution([FindPackageShare("hba"), "rviz", "hba.rviz"])
    config_path = PathJoinSubstitution([FindPackageShare("hba"), "config", "hba.yaml"])
    use_rviz = LaunchConfiguration("use_rviz")

    return launch.LaunchDescription(
        [
            # Map refinement is a service-driven batch step (/hba/refine_map ->
            # /hba/save_poses) that is normally run headless, so rviz2 is opt-in: the
            # previously unconditional rviz2 node made the documented flow impossible
            # without a display and cluttered the graph the node is measured on.
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="start rviz2 for visual debugging of the refined map",
            ),
            launch_ros.actions.Node(
                package="hba",
                namespace="hba",
                executable="hba_node",
                name="hba",
                output="screen",
                parameters=[{"config_path": config_path}],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="hba",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
