"""执行层独立启动。

真机部署时，执行层通常和感知层分开跑（甚至在不同机器上）：
    ros2 launch guide_executor executor.launch.py

参数：
  output_topic      底盘速度指令话题（默认 /chassis/velocity）
                    真机接 ros2_control diff_drive_controller 时改成 /cmd_vel
  watchdog_timeout  指令看门狗超时（秒），超时后自行停车
  max_speed         正常直行速度（m/s）
  caution_speed     MODE_CAUTION 时的限速（m/s）
  turn_speed        转向角速度（rad/s）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """返回执行层启动描述。"""
    return LaunchDescription([
        DeclareLaunchArgument('output_topic', default_value='/chassis/velocity',
                              description='底盘速度指令话题'),
        DeclareLaunchArgument('watchdog_timeout', default_value='0.5',
                              description='指令看门狗超时（秒）'),
        DeclareLaunchArgument('max_speed', default_value='0.30',
                              description='正常直行速度（m/s）'),
        DeclareLaunchArgument('caution_speed', default_value='0.15',
                              description='MODE_CAUTION 限速（m/s）'),
        DeclareLaunchArgument('turn_speed', default_value='0.40',
                              description='转向角速度（rad/s）'),

        Node(
            package='guide_executor',
            executable='executor_node',
            name='executor',
            output='screen',
            parameters=[{
                'output_topic': ParameterValue(
                    LaunchConfiguration('output_topic'), value_type=str),
                'watchdog_timeout': ParameterValue(
                    LaunchConfiguration('watchdog_timeout'), value_type=float),
                'max_speed': ParameterValue(
                    LaunchConfiguration('max_speed'), value_type=float),
                'caution_speed': ParameterValue(
                    LaunchConfiguration('caution_speed'), value_type=float),
                'turn_speed': ParameterValue(
                    LaunchConfiguration('turn_speed'), value_type=float),
            }],
        ),
    ])
