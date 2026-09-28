import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')  
    pkg_robot_2d_nav = "robot_2d_navigation"  

    # 声明参数，使用robot_2d_navigation包的配置文件路径
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 声明控制rviz启动的参数
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', 
        default_value='true',  # 默认启动rviz
        description='Whether to launch rviz2 (true: launch, false: close)'
    )
    use_rviz = LaunchConfiguration('use_rviz') 
    #  找到栅格地图文件路径
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value='default_map',
        description='Full path to map yaml file. Can point to any subdirectory under /workspace/data/'
    )
    map_yaml_file = PathJoinSubstitution([
        '/home',
        'flyos-universe-ros2',
        'data',
        LaunchConfiguration('map'),  
        'map.yaml'
    ])

    # 导航参数文件路径
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare(pkg_robot_2d_nav), "config", "nav2_smacl_params.yaml"]
    )
    
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # 声明导航启动文件，传入官方启动文件路径和参数
    nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file}.items(),
        )

    # rviz节点
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(use_rviz))

    # 静态TF转换节点（body→base_link）
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_base_link_tf",
        arguments=["0", "0", "0", "0", "0", "0", "body", "base_link"],
        output="screen"
    )

    # 位姿输出节点
    map_pose_node = Node(
        package="robot_pose",
        executable="map_pose_publisher",
        name="map_pose_publisher"

    )

    # 速度平滑节点
    cmd_smooth_node = Node(
        package="cmd_vel_smoother",
        executable="cmd_vel_smoother_node",
        name="cmd_vel_smoother_node"
    )

    return LaunchDescription([
        nav2_bringup_launch,
        rviz_node,
        static_tf_node,
        map_pose_node,
        cmd_smooth_node
    ])
