import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable, ThisLaunchFileDir, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

################### Livox驱动参数（原第一个脚本） ###################
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
################### 通用参数声明（统一map/use_sim_time） ###################
def generate_launch_description():
    # 1. 统一声明核心参数（删除重复的map参数声明）
    declare_data_dir_arg = DeclareLaunchArgument(
        name="data_dir",
        default_value=EnvironmentVariable('FLYOS_DATA_DIR', default_value='/home/flyos-universe-ros2/data'),
        description="地图/航点数据根目录(可由环境变量 FLYOS_DATA_DIR 覆盖)，迁移到其他机器/用户时按需传入 data_dir:=..."
    )
    declare_map_arg = DeclareLaunchArgument(
        name="map",
        default_value="default_map",
        description="Name of the map subdirectory (under data_dir) to load PCD/map.yaml from"
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_name = LaunchConfiguration('map')
    data_dir = LaunchConfiguration('data_dir')

    # body→base_link 静态外参(默认全 0；迁移到雷达/IMU 非原点安装的机器人时按需传入)
    declare_body_ext_args = []
    for _name in ['body_base_x', 'body_base_y', 'body_base_z', 'body_base_roll', 'body_base_pitch', 'body_base_yaw']:
        declare_body_ext_args.append(DeclareLaunchArgument(_name, default_value='0.0'))
    body_base_x = LaunchConfiguration('body_base_x')
    body_base_y = LaunchConfiguration('body_base_y')
    body_base_z = LaunchConfiguration('body_base_z')
    body_base_roll = LaunchConfiguration('body_base_roll')
    body_base_pitch = LaunchConfiguration('body_base_pitch')
    body_base_yaw = LaunchConfiguration('body_base_yaw')

    # 2. 原第一个脚本的节点：Livox+FAST-LIO2+ICP重定位+彩色点云显示
    ## Livox驱动节点
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )
    ## fastlio2建图节点
    sensing_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )
    lio_node = Node(
        package="fastlio2",
        namespace="mapping",
        executable="lio_node",
        name="mapping_node",
        output="screen",
        parameters=[{"config_path": sensing_config_path.perform(launch.LaunchContext())}]
    )
    ## ICP重定位节点
    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer.yaml"]
    )
    localizer_pcd_path = PathJoinSubstitution([
        data_dir,
        map_name,
        "map.pcd"
    ])
    localizer_node = Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[{
            "config_path": localizer_config_path.perform(launch.LaunchContext()),
            "pcd_path": localizer_pcd_path
        }]
    )

    ## 彩色点云显示节点
    full_colored_map_path = PathJoinSubstitution([
        data_dir,
        map_name,
        "colored_map.pcd"
    ])
    colored_pcd_display_node = Node(
        package="robot_2d_navigation",
        executable="pcd_display_node",
        name="pcd_display_node",
        output="screen",
        parameters=[
            {"file_path": full_colored_map_path},
            {"frame_id": "map"}
        ]
    )

    # 3. 原第二个脚本的节点：Nav2导航+静态TF+位姿输出+速度平滑
    ## Nav2相关路径配置
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')  
    pkg_robot_2d_nav = "robot_2d_navigation"  
    ## 栅格地图yaml路径（和第一个脚本的map参数统一）
    map_yaml_file = PathJoinSubstitution([
        data_dir,
        map_name,
        'map.yaml'
    ])
    ## 导航参数文件路径
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare(pkg_robot_2d_nav), "config", "nav2_smac_params2.yaml"]
    )
    ## 启动Nav2官方launch（传递统一参数）
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file
        }.items(),
    )
    ## 静态TF转换节点（body→base_link），外参可配置
    # 使用 Humble 支持的具名参数（--x/--y/--z/--roll/--pitch/--yaw/--frame-id/--child-frame-id）。
    # 位置参数形式是 `x y z yaw pitch roll parent child`：旧写法按 `x y z yaw roll pitch` 传入，
    # 把 body_base_roll 放进了 pitch 槽、body_base_pitch 放进了 roll 槽。默认全 0 时不可见，
    # 一旦传入非零安装倾角就会发布错误（roll/pitch 互换）的 body→base_link 旋转。
    # 具名参数与顺序无关，从根上消除该歧义。
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_base_link_tf",
        arguments=[
            "--x", body_base_x, "--y", body_base_y, "--z", body_base_z,
            "--roll", body_base_roll, "--pitch", body_base_pitch, "--yaw", body_base_yaw,
            "--frame-id", "body", "--child-frame-id", "base_link",
        ],
        output="screen"
    )
    ## 位姿输出节点
    map_pose_node = Node(
        package="robot_pose",
        executable="map_pose_publisher",
        name="map_pose_publisher",
        output="screen"
    )
    ## 速度平滑节点
    cmd_smooth_node = Node(
        package="cmd_vel_smoother",
        executable="cmd_vel_smoother_node",
        name="cmd_vel_smoother_node",
        output="screen"
    )
    ##航点管理节点
    multi_goal_manager_node = Node(
        package='g1_multi_goal_manager',
        executable='g1_multi_goal_manager_node.py',
        name='g1_multi_goal_manager_node',
        output='screen',
        # 通过parameters参数传递map参数
        parameters=[{
            'map': map_name  # 使用当前launch的map参数
        }]
    )
    ## 移动控制节点
    movement_gate_node = Node(
    package='g1_movement_gate',
    executable='g1_movement_gate_node',
    name='g1_movement_gate_node',
    output='screen',
    )
    ## ✅ 新增势场排斥节点（
    obstacle_config_path = PathJoinSubstitution(
        [FindPackageShare("lidar_preprocessor"), "config", "config.yaml"]
    )
    obstacle_avoidance_node = Node(
        package='lidar_preprocessor',
        executable='preprocessor',
        name='preprocessor',
        output='screen',
        parameters=[{
            "config_path": obstacle_config_path
        }]
    )     

    rviz_cfg = PathJoinSubstitution([FindPackageShare("localizer"), "rviz", "localizer.rviz"])
    rviz_node = Node(
        package="rviz2",
        # namespace="localizer",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
    )

    # 5. 整合所有节点/参数
    return LaunchDescription([
        declare_data_dir_arg,
        declare_map_arg,
        declare_use_sim_time_arg,
        *declare_body_ext_args,
        livox_driver,
        lio_node,
        multi_goal_manager_node,
        localizer_node,
        colored_pcd_display_node,
        nav2_bringup_launch,
        static_tf_node,
        map_pose_node,
        obstacle_avoidance_node,
        cmd_smooth_node,
        movement_gate_node,
        #rviz_node
    ])
