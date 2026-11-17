"""上层应用独立启动。

    ros2 launch guide_upper upper.launch.py

上游需要 safety_supervisor 在发 /search_hint，本节点只做提示消费。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """返回上层应用启动描述。"""
    return LaunchDescription([
        DeclareLaunchArgument('publish_topic', default_value='/search_hint_text',
                              description='提示文本输出话题'),
        DeclareLaunchArgument('throttle_sec', default_value='5.0',
                              description='同一提示的最小重复间隔（秒）'),

        Node(
            package='guide_upper',
            executable='search_hint_node',
            name='search_hint',
            output='screen',
            parameters=[{
                'publish_topic': ParameterValue(
                    LaunchConfiguration('publish_topic'), value_type=str),
                'throttle_sec': ParameterValue(
                    LaunchConfiguration('throttle_sec'), value_type=float),
            }],
        ),
    ])
