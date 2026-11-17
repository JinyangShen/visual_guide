"""轨迹记录节点

把 SLAM 输出的 /odom 记录成可回溯的轨迹，用于：
  1. 急停事件的事后分析（当时在哪、怎么走过来的）
  2. 0.2m / 0.5m 等融合阈值的实车数据调参
  3. rviz2 / Foxglove 直接回放
  4. 为后续"盲道丢失后回归搜索"提供历史轨迹

输入：
  /odom                (nav_msgs/Odometry)

输出：
  /guide_trajectory    (nav_msgs/Path)   轨迹点序列
  /trajectory_length   (std_msgs/Int32) 当前轨迹点数

设计要点：
  - 环形缓冲，节点内存占用与运行时长无关（默认保留最近 3000 点）
  - 可选落盘为 CSV，掉电后仍可分析
  - 与安全链路完全解耦：本节点崩溃不会影响行走决策
"""

import math
import os
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        # 轨迹缓冲长度（按 /odom 10Hz 计，3000 点约 5 分钟）
        self.max_points = self.declare_parameter('max_points', 3000).value
        # 轨迹发布频率（Hz）：比采样频率低，避免占用带宽
        self.publish_rate = self.declare_parameter('publish_rate', 1.0).value
        # 是否记录到 CSV 文件
        self.record_csv = self.declare_parameter('record_csv', False).value
        self.csv_path = self.declare_parameter(
            'csv_path', os.path.expanduser('~/guide_trajectory.csv')).value
        # 轨迹坐标系（与 /odom 的 frame_id 一致）
        self.frame_id = self.declare_parameter('frame_id', 'odom').value

        self.points = deque(maxlen=self.max_points)
        self.csv_file = None

        if self.record_csv:
            try:
                self.csv_file = open(self.csv_path, 'w', encoding='utf-8', newline='')
                self.csv_file.write('timestamp,x,y,yaw\n')
                self.get_logger().info(f'recording trajectory to {self.csv_path}')
            except OSError as e:
                self.get_logger().error(f'cannot open csv {self.csv_path}: {e}')
                self.csv_file = None

        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.pub = self.create_publisher(Path, '/guide_trajectory', 10)
        self.len_pub = self.create_publisher(Int32, '/trajectory_length', 10)
        self.create_timer(1.0 / self.publish_rate, self.on_timer)

        self.get_logger().info(
            f'trajectory_recorder started (max_points={self.max_points}, '
            f'publish_rate={self.publish_rate}Hz, csv={self.record_csv})')

    # ============================================================
    def on_odom(self, msg: Odometry):
        stamp = msg.header.stamp if msg.header.stamp.sec or msg.header.stamp.nanosec \
            else self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose
        self.points.append(pose)

        if self.csv_file is not None:
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            t = stamp.sec + stamp.nanosec * 1e-9
            try:
                self.csv_file.write(f'{t:.3f},{msg.pose.pose.position.x:.4f},'
                                    f'{msg.pose.pose.position.y:.4f},{yaw:.4f}\n')
            except (OSError, ValueError):
                self.csv_file = None

    def on_timer(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        path.poses = list(self.points)
        self.pub.publish(path)
        self.len_pub.publish(Int32(data=len(self.points)))

    def destroy_node(self):
        if self.csv_file is not None:
            try:
                self.csv_file.close()
            except OSError:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
