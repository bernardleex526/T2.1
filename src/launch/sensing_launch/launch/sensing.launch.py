import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

################### user configure parameters for ros2 start ###################
# Livox驱动参数
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

user_config_path = PathJoinSubstitution(
    [FindPackageShare("livox_ros_driver2"), "config", "MID360_config.json"]
)

# Octomap参数 - 优化版
input_cloud_topic = '/mapping/world_cloud'
resolution = 0.2
octomap_frame_id = 'odom'
base_frame_id = 'body'
height_map = False
colored_map = False
color_factor = 0.8
filter_ground = False
filter_speckles = False
ground_filter_distance = 0.04
ground_filter_angle = 0.15
ground_filter_plane_distance = 0.07
compress_map = True
incremental_2D_projection = False
sensor_model_max_range = 8.0
sensor_model_hit = 0.7
sensor_model_miss = 0.4
sensor_model_min = 0.12
sensor_model_max = 0.97
color_r = 0.0
color_g = 0.0
color_b = 1.0
color_a = 1.0
color_free_r = 0.0
color_free_g = 1.0
color_free_b = 0.0
color_free_a = 1.0
publish_free_space = False
max_range = 10.0
publish_freq_map = 1.0
latch = False
track_changes = False
listen_changes = False
max_z = 2.0
min_z = -0.5

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]

octomap_params = [
    {"resolution": resolution},
    {"frame_id": octomap_frame_id},
    {"base_frame_id": base_frame_id},
    {"height_map": height_map},
    {"colored_map": colored_map},
    {"color_factor": color_factor},
    {"filter_ground": filter_ground},
    {"filter_speckles": filter_speckles},
    {"ground_filter/distance": ground_filter_distance},
    {"ground_filter/angle": ground_filter_angle},
    {"ground_filter/plane_distance": ground_filter_plane_distance},
    {"compress_map": compress_map},
    {"incremental_2D_projection": incremental_2D_projection},
    {"sensor_model/max_range": sensor_model_max_range},
    {"sensor_model/hit": sensor_model_hit},
    {"sensor_model/miss": sensor_model_miss},
    {"sensor_model/min": sensor_model_min},
    {"sensor_model/max": sensor_model_max},
    {"color/r": color_r},
    {"color/g": color_g},
    {"color/b": color_b},
    {"color/a": color_a},
    {"color_free/r": color_free_r},
    {"color_free/g": color_free_g},
    {"color_free/b": color_free_b},
    {"color_free/a": color_free_a},
    {"publish_free_space": publish_free_space},
    {"max_range": max_range},
    {"publish_freq": publish_freq_map},
    {"latch": latch},
    {"track_changes": track_changes},
    {"listen_changes": listen_changes},
    {"max_z": max_z},
    {"min_z": min_z}
]

def generate_launch_description():
    lidar_vendor = LaunchConfiguration('lidar_vendor', default='robosense')
    config_file = LaunchConfiguration('config_file', default='airy.yaml')

    is_robosense = PythonExpression(["'", lidar_vendor, "' == 'robosense'"])
    is_livox = PythonExpression(["'", lidar_vendor, "' == 'livox'"])

    # 1. RoboSense 点云转换适配节点
    rs_adapter_node = Node(
        package='rs_to_fastlio',
        executable='rs_to_fastlio_node',
        name='rs_to_fastlio_node',
        output='screen',
        condition=IfCondition(is_robosense),
        parameters=[{
            'input_topic': '/rslidar_points',
            'output_topic': '/rslidar_points_adapted',
            'target_frame': 'lidar_link',
            'min_range': 0.3,
            'max_range': 50.0
        }]
    )

    # 2. Livox mid360驱动
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        condition=IfCondition(is_livox),
        parameters=livox_ros2_params
    )

    # 3. 四足机器人 REP-105 TF 树发布 (base_link -> imu_link / lidar_link)
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('quadruped_description'), 'launch', 'rsp.launch.py'])
        )
    )

    # 4. 建图节点配置路径
    sensing_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", config_file]
    )

    # 5. lio建图节点
    lio_node = Node(
        package="fastlio2",
        namespace="mapping",
        executable="lio_node",
        name="mapping_node",
        output="screen",
        parameters=[{"config_path": sensing_config_path}]
    )

    # 6. pgo配置路径与节点
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo.yaml"]
    )
    pgo_node = Node(
        package="pgo",
        namespace="pgo",
        executable="pgo_node",
        name="pgo_node",
        output="screen",
        parameters=[{"config_path": pgo_config_path}]
    )

    # 7. octomap_server节点
    octomap_server = Node(
        package='octomap_server2',
        executable='octomap_server',
        name='octomap_server_node',
        namespace="octomap",
        output='screen',
        remappings=[('cloud_in', input_cloud_topic)],
        parameters=octomap_params
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_vendor', default_value='robosense',
                              description='LiDAR vendor: robosense or livox'),
        DeclareLaunchArgument('config_file', default_value='airy.yaml',
                              description='FastLIO2 config file name (e.g. airy.yaml, odin1.yaml, lio.yaml)'),
        rsp_launch,
        rs_adapter_node,
        livox_driver,
        lio_node,
        pgo_node,
        octomap_server
    ])
