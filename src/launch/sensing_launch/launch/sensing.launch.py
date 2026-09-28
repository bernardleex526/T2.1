import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
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
resolution = 0.2  # 降低分辨率，从0.05提高到0.15，性能提升27倍
octomap_frame_id = 'odom'
base_frame_id = 'body'  # 建图阶段 lio 只发布 odom→body，此处用 body 而非 base_footprint 才有 TF
height_map = False   # 关闭高度图，减少计算
colored_map = False  # 关闭彩色地图，减少计算
color_factor = 0.8
filter_ground = False
filter_speckles = False
ground_filter_distance = 0.04
ground_filter_angle = 0.15
ground_filter_plane_distance = 0.07
compress_map = True
incremental_2D_projection = False  # 关闭增量投影，减少计算
sensor_model_max_range = 8.0  # 限制处理范围，减少数据量
sensor_model_hit = 0.7
sensor_model_miss = 0.4
sensor_model_min = 0.12
sensor_model_max = 0.97
color_r = 0.0
color_g = 0.0
color_b = 1.0
color_a = 1.0
color_free_r = 0.0
color_free_g = 0.0
color_free_b = 1.0
color_free_a = 1.0
publish_free_space = False

# 新增性能优化参数
max_range = 8.0  # 限制点云处理范围
publish_freq_map = 1.0  # 降低地图发布频率
latch = False  # 不保持最新消息
track_changes = False  # 不跟踪变化
listen_changes = False  # 不监听变化
max_z = 2.0  # 最大高度限制
min_z = 0.2  # 最小高度限制
################### user configure parameters for ros2 end #####################

# Livox参数配置
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

# Octomap参数配置 - 优化版
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
    # 新增性能参数
    {"max_range": max_range},
    {"publish_freq": publish_freq_map},
    {"latch": latch},
    {"track_changes": track_changes},
    {"listen_changes": listen_changes},
    {"max_z": max_z},
    {"min_z": min_z}
]


def generate_launch_description():
    
    # mid360驱动
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    # 建图节点配置路径
    sensing_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )

    # lio建图节点
    lio_node = Node(
        package="fastlio2",
        namespace="mapping",
        executable="lio_node",
        name="mapping_node",
        output="screen",
        parameters=[{"config_path": sensing_config_path.perform(launch.LaunchContext())}]
    )

    # pgo配置路径
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo.yaml"]
    )

    # pgo节点
    pgo_node = Node(
        package="pgo",
        namespace="pgo",
        executable="pgo_node",
        name="pgo_node",
        output="screen",
        parameters=[{"config_path": pgo_config_path.perform(launch.LaunchContext())}]
    )

    # octomap_server节点 - 优化版本
    octomap_server = Node(
        package='octomap_server2',
        executable='octomap_server',
        name='octomap_server_node',
        namespace="octomap",
        output='screen',
        remappings=[('cloud_in', input_cloud_topic)],
        parameters=octomap_params
    )

    # rviz配置路径
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "fastlio2.rviz"]
    )

    # 启动rviz
    rviz_screen = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
    )

    return LaunchDescription([
        livox_driver,
        lio_node,
        pgo_node,
        octomap_server,  # 启用 octomap 以产出 /octomap/projected_map，供 map_saver 生成 2D 栅格地图
        #rviz_screen
    ])