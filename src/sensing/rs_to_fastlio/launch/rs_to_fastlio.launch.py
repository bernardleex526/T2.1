import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic', default='/rslidar_points')
    output_topic = LaunchConfiguration('output_topic', default='/sensing/lidar/pointcloud')
    target_frame = LaunchConfiguration('target_frame', default='lidar_link')
    min_range = LaunchConfiguration('min_range', default='0.3')
    max_range = LaunchConfiguration('max_range', default='50.0')

    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value=input_topic),
        DeclareLaunchArgument('output_topic', default_value=output_topic),
        DeclareLaunchArgument('target_frame', default_value=target_frame),
        DeclareLaunchArgument('min_range', default_value=min_range),
        DeclareLaunchArgument('max_range', default_value=max_range),

        Node(
            package='rs_to_fastlio',
            executable='rs_to_fastlio_node',
            name='rs_to_fastlio_node',
            output='screen',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': output_topic,
                'target_frame': target_frame,
                'min_range': min_range,
                'max_range': max_range,
            }]
        )
    ])
