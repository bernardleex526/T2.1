from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    return LaunchDescription([
        # 启动地图保存服务器
        LifecycleNode(
            package='nav2_map_server',
            executable='map_saver_server',
            name='map_saver',
            namespace='',
            output='screen'
        ),
        # 启动生命周期管理器，自动激活map_saver
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_saver',
            output='screen',
            parameters=[{
                'node_names': ['map_saver'],  # 指定要管理的节点名
                'autostart': True  # 自动启动所有节点的生命周期
            }]
        )
    ])