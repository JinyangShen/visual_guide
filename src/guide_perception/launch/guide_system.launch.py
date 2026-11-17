"""
导盲感知系统一键启动（ROS 2 Humble / RDK X5）

启动内容：
  1. guide_node              视觉感知（盲道 HSV + DNN 障碍物）
  2. ultrasonic_node         超声波近距兜底（默认模拟模式）
  3. safety_supervisor_node  独立安全仲裁（感知崩溃时仍能输出停止指令）
  4. executor_node           执行层（只认 command，带 0.5s 指令看门狗）
  5. search_hint_node        上层提示（消费 /search_hint，转中文提示）
  6. trajectory_recorder_node 轨迹记录（/guide_trajectory，可选落盘 CSV）
  7. rtabmap_ros             视觉 SLAM 建图（RGB-D 模式）

使用示例：
  # 开发调试（无相机/无硬件，超声波模拟）
  ros2 launch guide_perception guide_system.launch.py

  # 真机运行（接 RealSense D435i + 真实超声波）
  ros2 launch guide_perception guide_system.launch.py \
      simulate_ultrasonic:=false \
      rgb_topic:=/camera/color/image_raw \
      depth_topic:=/camera/aligned_depth_to_color/image_raw \
      camera_info_topic:=/camera/color/camera_info

说明：
  - use_supervisor:=true（默认）时 guide_node 处于 external 模式，
    只发布 /perception_state，最终指令由安全仲裁节点发布；
    设为 false 则 guide_node 回到 inline 模式自行下发指令。
  - RTAB-Map 通过 IncludeLaunchDescription 复用其官方 launch。
  - 2D 激光方案（slam_toolbox）已安装但默认不启用，当前用超声波 + 摄像头。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _perception_nodes(context):
    """根据开关决定感知/仲裁/轨迹节点的组合。

    use_supervisor=true  -> guide_node 走 external 模式（只发感知结果），
                           由 safety_supervisor 仲裁并发布 /guide_command
    use_supervisor=false -> guide_node 走 inline 模式，直接发布 /guide_command
    """
    use_supervisor = LaunchConfiguration('use_supervisor').perform(context) == 'true'
    use_trajectory = LaunchConfiguration('use_trajectory').perform(context) == 'true'
    fusion_mode = 'external' if use_supervisor else 'inline'

    nodes = [
        # ---- 视觉感知节点 ----
        Node(
            package='guide_perception',
            executable='guide_node',
            name='guide_perception_node',
            output='screen',
            parameters=[{
                # 可按现场盲道颜色微调
                'h_min': 16, 'h_max': 40, 's_min': 80, 'v_min': 80,
                'ultra_stop_dist': 0.2,
                'ultra_caution_dist': 0.5,
                'ultra_timeout': 1.0,
                'fusion_mode': fusion_mode,
            }],
        ),
    ]

    if use_supervisor:
        # ---- 独立安全仲裁节点 ----
        # 感知进程崩溃/图像超时 -> 仲裁节点独立输出 MODE_STOP
        nodes.append(Node(
            package='guide_perception',
            executable='safety_supervisor_node',
            name='safety_supervisor',
            output='screen',
            parameters=[{
                'ultra_stop_dist': 0.2,
                'ultra_caution_dist': 0.5,
                'ultra_timeout': 1.0,
                'perception_timeout': 1.0,
                'dead_zone': 0.15,
                'decision_rate': 10.0,
                # 停滞检测（里程计参与决策）
                'stuck_window': 3.0,
                'stuck_threshold': 0.05,
                # 盲道丢失后的回归搜索
                'max_regression_distance': 5.0,
                'search_angle_threshold': 0.35,
            }],
        ))

    if use_trajectory:
        # ---- 轨迹记录节点 ----
        nodes.append(Node(
            package='guide_perception',
            executable='trajectory_recorder_node',
            name='trajectory_recorder',
            output='screen',
            parameters=[{
                'max_points': 3000,
                'publish_rate': 1.0,
                'record_csv': LaunchConfiguration('record_trajectory_csv'),
                'csv_path': LaunchConfiguration('trajectory_csv_path'),
            }],
        ))

    return nodes


def generate_launch_description():
    # ---- 可配置参数 ----
    simulate_ultrasonic = LaunchConfiguration('simulate_ultrasonic')
    use_rtabmap = LaunchConfiguration('use_rtabmap')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    publish_camera_tf = LaunchConfiguration('publish_camera_tf')
    use_executor = LaunchConfiguration('use_executor')
    use_upper = LaunchConfiguration('use_upper')
    use_mock_chassis = LaunchConfiguration('use_mock_chassis')
    executor_output_topic = LaunchConfiguration('executor_output_topic')
    executor_watchdog_timeout = LaunchConfiguration('executor_watchdog_timeout')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    return LaunchDescription([
        DeclareLaunchArgument('simulate_ultrasonic', default_value='true',
                              description='超声波模拟数据开关（无硬件时保持 true）'),
        DeclareLaunchArgument('use_supervisor', default_value='true',
                              description='是否启用独立安全仲裁节点（建议保持 true）'),
        DeclareLaunchArgument('use_trajectory', default_value='true',
                              description='是否启用轨迹记录节点'),
        DeclareLaunchArgument('record_trajectory_csv', default_value='false',
                              description='轨迹是否落盘为 CSV（用于事后分析）'),
        DeclareLaunchArgument('trajectory_csv_path',
                              default_value='/home/jinyangshen/guide_trajectory.csv',
                              description='轨迹 CSV 输出路径'),
        DeclareLaunchArgument('use_rtabmap', default_value='true',
                              description='是否启动 RTAB-Map 建图'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false',
                              description='是否启动 RTAB-Map GUI（无显示器环境保持 false）'),
        DeclareLaunchArgument('publish_camera_tf', default_value='true',
                              description='是否发布 camera_link->base_link 静态 TF 兜底'),
        DeclareLaunchArgument('use_executor', default_value='true',
                              description='是否启动执行层（真机接底盘时保持 true）'),
        DeclareLaunchArgument('use_mock_chassis', default_value='false',
                              description='是否启动模拟底盘（开发验证用，真机保持 false）'),
        DeclareLaunchArgument('use_upper', default_value='true',
                              description='是否启动上层提示节点（消费 /search_hint）'),
        DeclareLaunchArgument('executor_output_topic',
                              default_value='/chassis/velocity',
                              description='执行层输出的底盘速度话题'),
        DeclareLaunchArgument('executor_watchdog_timeout', default_value='0.5',
                              description='执行层指令看门狗超时（秒）'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic',
                              default_value='/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),

        # ---- 1. 视觉感知 / 安全仲裁 / 轨迹记录（按开关组合） ----
        OpaqueFunction(function=_perception_nodes),

        # ---- 2. 执行层：只认 command 字段，自带 0.5s 指令看门狗 ----
        # 契约见 guide_interfaces/EXECUTION_CONTRACT.md
        Node(
            package='guide_executor',
            executable='executor_node',
            name='executor',
            output='screen',
            parameters=[{
                'output_topic': executor_output_topic,
                'watchdog_timeout': ParameterValue(
                    executor_watchdog_timeout, value_type=float),
                'max_speed': 0.30,
                'caution_speed': 0.15,
                'turn_speed': 0.40,
            }],
            condition=IfCondition(use_executor),
        ),

        # ---- 3. 上层提示节点：消费 /search_hint，转成中文提示 ----
        # 执行层不订阅 /search_hint，这里是方案 A 里"给上层的建议"的消费端
        Node(
            package='guide_upper',
            executable='search_hint_node',
            name='search_hint',
            output='screen',
            parameters=[{
                'throttle_sec': 5.0,
            }],
            condition=IfCondition(use_upper),
        ),

        # ---- 4. 模拟底盘（仅开发验证用；真机不要启动，避免和 rtabmap 抢 /odom） ----
        Node(
            package='guide_executor',
            executable='mock_chassis_node',
            name='mock_chassis',
            output='screen',
            parameters=[{
                'velocity_topic': executor_output_topic,
            }],
            condition=IfCondition(use_mock_chassis),
        ),

        # ---- 5. 超声波节点 ----
        Node(
            package='guide_perception',
            executable='ultrasonic_node',
            name='ultrasonic_node',
            output='screen',
            parameters=[{
                'simulate': simulate_ultrasonic,
                'publish_rate': 10.0,
                # 真机时按实际接线修改：
                # 'i2c_bus': 1, 'i2c_addr': 0x70,
                # 'gpio_trig': 23, 'gpio_echo': 24,
            }],
        ),

        # ---- 6. 相机坐标系静态 TF ----
        # 建图节点需要 camera_link -> base_link 的 TF；
        # 相机驱动未发布时由这里兜底，避免 rgbd_odometry 因缺 TF 而空转
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            output='log',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
            condition=IfCondition(publish_camera_tf),
        ),

        # ---- 7. RTAB-Map 视觉建图（RGB-D 模式） ----
        # 需要相机驱动先发布 RGB / Depth / CameraInfo 三个话题
        # odom 话题由 rtabmap 的视觉里程计输出，自动喂给 guide_node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('rtabmap_launch'),
                    'launch',
                    'rtabmap.launch.py',
                ])
            ),
            condition=IfCondition(use_rtabmap),
            launch_arguments={
                'rgb_topic': rgb_topic,
                'depth_topic': depth_topic,
                'camera_info_topic': camera_info_topic,
                'approx_sync': 'true',
                'subscribe_rgbd': 'false',
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                # 关键：把 rtabmap 的定位输出重映射到 /odom，与 guide_node 订阅一致
                # （默认是 /rtabmap/odom，guide_node 收不到）
                'odom_topic': '/odom',
                'vo_frame_id': 'odom',
                'map_topic': '/map',
                # RDK X5 算力有限，降低特征数以保帧率。
                # 注意：args 会同时传给 rtabmap 与 rgbd_odometry 两个节点，
                # 因此只能使用两个节点都声明过的参数，否则节点会因
                # "parameter ... cannot be set because it was not declared" 崩溃。
                'args': '--Kp/MaxFeatures 400 --Vis/MaxFeatures 400',
                # 里程计专属参数走 odom_args（只传给 rgbd_odometry）
                # 0 = Frame-to-Map 视觉里程计
                'odom_args': '--Odom/Strategy 0',
                # 无显示器环境（x86 开发机 / RDK X5 SSH 调试）不能开 GUI
                'rtabmap_viz': rtabmap_viz,
                'rviz': 'false',
                'use_sim_time': 'false',
            }.items(),
        ),
    ])
