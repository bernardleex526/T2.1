import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 声明和导航Launch一致的核心参数（map参数+use_sim_time，保持风格统一）
    declare_map_arg = DeclareLaunchArgument(
        name="map",
        default_value="default_map",
        description="Name of the map subdirectory (under /home/flyos-universe-ros2/data/) to load map.pcd from"
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_thre_z_min_arg = DeclareLaunchArgument(
        'thre_z_min',
        default_value='0.3',
        description='直通滤波最小Z高度'
    )
    declare_thre_z_max_arg = DeclareLaunchArgument(
        'thre_z_max',
        default_value='2.0',
        description='直通滤波最大Z高度'
    )
    declare_thre_radius_arg = DeclareLaunchArgument(
        'thre_radius',
        default_value='0.1',
        description='半径滤波搜索半径'
    )
    declare_thres_point_count_arg = DeclareLaunchArgument(
        'thres_point_count',
        default_value='10',
        description='半径滤波邻域点数阈值'
    )
    declare_map_resolution_arg = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.05',
        description='栅格地图分辨率（m/栅格）'
    )
    declare_map_topic_name_arg = DeclareLaunchArgument(
        'map_topic_name',
        default_value='map',
        description='发布的栅格地图话题名'
    )

    # 2. 定义参数变量（和导航Launch逻辑一致）
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_name = LaunchConfiguration('map')  # 对应map:=xxx的参数
    thre_z_min = LaunchConfiguration('thre_z_min')
    thre_z_max = LaunchConfiguration('thre_z_max')
    thre_radius = LaunchConfiguration('thre_radius')
    thres_point_count = LaunchConfiguration('thres_point_count')
    map_resolution = LaunchConfiguration('map_resolution')
    map_topic_name = LaunchConfiguration('map_topic_name')

    # 3. 拼接PCD文件路径（和导航Launch的pcd_path逻辑完全一致）
    # 路径：~/flyos-universe-ros2/data/${map_name}/map.pcd
    pcd_file_directory = PathJoinSubstitution([
        '/home',
        "flyos-universe-ros2", 
        "data",
        map_name,
        ""  # 末尾空字符串保证路径带/，无需手动加
    ])
    pcd_file_name = "map"  # 固定文件名map.pcd，仅目录随map参数变化

    # 4. 配置pcd2grid节点（传递拼接后的路径+统一参数）
    pcd2grid_node = Node(
        package='pcd2grid',          # 功能包名
        executable='pcd2grid_node',  # 可执行文件名
        name='pcd2grid',             # 节点名
        output='screen',             # 日志输出到终端
        parameters=[{                # 参数传递（和导航Launch风格一致）
            'file_directory': pcd_file_directory,  # 自动拼接的目录
            'file_name': pcd_file_name,            # 固定为map（对应map.pcd）
            'thre_z_min': thre_z_min,
            'thre_z_max': thre_z_max,
            'thre_radius': thre_radius,
            'thres_point_count': thres_point_count,
            'map_resolution': map_resolution,
            'map_topic_name': map_topic_name,
            'use_sim_time': use_sim_time  # 可选：同步use_sim_time参数
        }]
    )

    # 5. （可选）整合map_saver保存地图（路径和map参数统一）
    from launch.actions import ExecuteProcess, TimerAction
    save_map_node = TimerAction(
        period=60.0,  # 延迟5秒，确保地图已发布
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                    '-t', map_topic_name,
                    '-f', PathJoinSubstitution([
                        '/home',
                        "flyos-universe-ros2", 
                        "data",
                        map_name,
                        "map"  # 保存为~/flyos-universe-ros2/data/xxx/map.pgm/.yaml
                    ]),
                    '--fmt', 'pgm',
                    '--mode', 'trinary'
                ],
                output='screen'
            )
        ]
    )

    # 6. 组装LaunchDescription
    ld = LaunchDescription()
    # 添加参数声明（先声明后使用）
    ld.add_action(declare_map_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_thre_z_min_arg)
    ld.add_action(declare_thre_z_max_arg)
    ld.add_action(declare_thre_radius_arg)
    ld.add_action(declare_thres_point_count_arg)
    ld.add_action(declare_map_resolution_arg)
    ld.add_action(declare_map_topic_name_arg)
    # 添加节点
    ld.add_action(pcd2grid_node)
    ld.add_action(save_map_node)  # 可选：如需自动保存地图则保留

    return ld
