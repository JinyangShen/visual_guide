"""安全仲裁节点（独立进程，不依赖感知节点存活）

设计目的：
  把"停止"这类安全决策从视觉感知节点中剥离。即使 guide_node（DNN/图像处理）
  崩溃、超时或输出异常，本节点仍能独立输出 MODE_STOP，保证执行器不会因为
  "收不到指令" 而继续前进。

输入：
  /perception_state   (guide_interfaces/PerceptionState)  视觉感知结果
  /ultrasonic/range   (sensor_msgs/Range)                  超声波近距测距
  /odom               (nav_msgs/Odometry)                  SLAM 定位输出

输出：
  /guide_command      (guide_interfaces/GuideCommand)       最终指令（仲裁结果）
  /safety_reason      (std_msgs/Int32)                      决策原因码（便于日志分析）

仲裁优先级（从高到低）：
  1. 感知数据超时            -> MODE_STOP（摄像头/感知失效兜底）
  2. 超声波 < 0.2 m          -> MODE_STOP（最高安全优先级，急停）
  3. 视觉障碍物              -> MODE_STOP
  4. 盲道完全丢失            -> MODE_STOP + 给出里程回归方向
  5. 里程计显示停滞          -> MODE_STOP（疑似卡住/被挡住）
  6. 超声波 0.2~0.5 m 且视觉前进 -> MODE_CAUTION（降速慢行）
  7. 正常循迹                -> MODE_FORWARD / LEFT / RIGHT

里程计（odom）在本节点中真正参与决策：
  - 盲道丢失时，用轨迹历史算出"往哪走能回到最后可信盲道位置"；
  - 连续一段时间位移几乎为零时，判定机器人卡住并停止。
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from guide_interfaces.msg import GuideCommand, PerceptionState, SearchHint


class SafetySupervisor(Node):
    def __init__(self):
        super().__init__('safety_supervisor')

        # ---- 超声波阈值 ----
        self.ultra_stop_dist = self.declare_parameter('ultra_stop_dist', 0.2).value
        self.ultra_caution_dist = self.declare_parameter('ultra_caution_dist', 0.5).value
        self.ultra_timeout = self.declare_parameter('ultra_timeout', 1.0).value

        # ---- 感知看门狗 ----
        # 超过该时间没有收到 /perception_state，视为视觉链路失效
        self.perception_timeout = self.declare_parameter('perception_timeout', 1.0).value

        # ---- 循迹控制 ----
        self.dead_zone = self.declare_parameter('dead_zone', 0.15).value
        self.decision_rate = self.declare_parameter('decision_rate', 10.0).value

        # ---- 停滞检测（里程计参与决策） ----
        self.stuck_window = self.declare_parameter('stuck_window', 3.0).value
        self.stuck_threshold = self.declare_parameter('stuck_threshold', 0.05).value

        # ---- 回归搜索 ----
        # 盲道丢失后，回归到最后一个"可信盲道位置"的最大距离（米）
        self.max_regression_distance = self.declare_parameter('max_regression_distance', 5.0).value
        # 航向差超过该值才建议转向回归，否则直行回归
        self.search_angle_threshold = self.declare_parameter('search_angle_threshold', 0.35).value

        # ---- 传感器缓存 ----
        self.perception = None
        self.perception_time = None
        self.ultra_dist = -1.0
        self.ultra_time = None
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_time = None

        # 轨迹历史：用于停滞检测与回归方向计算
        self.pose_history = deque(maxlen=1000)      # (t, x, y, yaw)
        self.trusted_pose = None                    # 最后一次确认看到盲道时的位姿
        self.trusted_time = None
        # 停滞锁存标志：卡住后保持停止，直到重新产生位移
        self.stuck_latched = False

        # 上一次下发的指令（用于停滞判断：只有本来在运动才可能"卡住"）
        self.last_command = GuideCommand.MODE_STOP

        self.create_subscription(PerceptionState, '/perception_state', self.on_perception, 10)
        self.create_subscription(Range, '/ultrasonic/range', self.on_ultrasonic, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        self.pub = self.create_publisher(GuideCommand, '/guide_command', 10)
        self.reason_pub = self.create_publisher(Int32, '/safety_reason', 10)
        # 搜索建议单独成话题：执行层不订阅，只给上层（人工遥控/导航）参考
        self.hint_pub = self.create_publisher(SearchHint, '/search_hint', 10)

        # 固定频率输出：即使传感器数据没变化也持续发布，兼作看门狗心跳
        self.create_timer(1.0 / self.decision_rate, self.on_timer)
        self.get_logger().info(
            f'safety_supervisor started (ultra_stop={self.ultra_stop_dist}m, '
            f'perception_timeout={self.perception_timeout}s, rate={self.decision_rate}Hz)')

    # ============================================================
    # 回调
    # ============================================================
    def on_perception(self, msg: PerceptionState):
        self.perception = msg
        self.perception_time = self.get_clock().now()
        # 看到盲道时记录"可信位姿"，作为盲道丢失后的回归目标
        if msg.blind_path_found and self.odom_time is not None:
            self.trusted_pose = (self.odom_x, self.odom_y, self.odom_yaw)
            self.trusted_time = self.get_clock().now()

    def on_ultrasonic(self, msg: Range):
        if math.isnan(msg.range) or msg.range < msg.min_range or msg.range > msg.max_range:
            return  # 无效量程，保持旧值等待超时降级
        self.ultra_dist = msg.range
        self.ultra_time = self.get_clock().now()

    def on_odom(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)
        now = self.get_clock().now()
        self.odom_time = now
        self.pose_history.append((now.nanoseconds * 1e-9, self.odom_x, self.odom_y, self.odom_yaw))

    # ============================================================
    # 有效性判断
    # ============================================================
    def _age(self, stamp):
        if stamp is None:
            return float('inf')
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def perception_valid(self):
        return self._age(self.perception_time) < self.perception_timeout

    def ultra_valid(self):
        return self._age(self.ultra_time) < self.ultra_timeout

    def odom_valid(self):
        return self._age(self.odom_time) < self.perception_timeout

    # ============================================================
    # 里程计参与的决策辅助
    # ============================================================
    def is_stuck(self):
        """检测机器人是否停滞：窗口内最大位移过小。

        采用"锁存"语义：一旦判定卡住就保持停止，直到机器人重新产生位移。
        否则会出现 STOP -> 下帧 last_command 变成 STOP 从而跳过检测 -> 又变
        FORWARD 的指令抖动，执行器将收到来回跳变的指令。
        """
        if len(self.pose_history) < 2:
            return self.stuck_latched
        now_t = self.get_clock().now().nanoseconds * 1e-9
        window = [p for p in self.pose_history if now_t - p[0] <= self.stuck_window]
        if len(window) < 2:
            return self.stuck_latched
        x0, y0 = window[0][1], window[0][2]
        max_move = 0.0
        for _, x, y, _yaw in window:
            max_move = max(max_move, math.hypot(x - x0, y - y0))
        if max_move >= self.stuck_threshold:
            self.stuck_latched = False   # 重新产生位移 -> 解锁，恢复行走
        else:
            self.stuck_latched = True    # 位移不足 -> 锁存为卡住
        return self.stuck_latched

    def compute_regression(self):
        """盲道丢失时计算回归方向与距离。

        原理：以"最后可信盲道位置"为目标点，换算到当前车体坐标系下的方位角，
        方位角为正说明目标在左侧 -> 建议向左回归。
        """
        if self.trusted_pose is None or not self.odom_valid():
            return 0, -1.0
        tx, ty, _ = self.trusted_pose
        dx = tx - self.odom_x
        dy = ty - self.odom_y
        dist = math.hypot(dx, dy)
        if dist > self.max_regression_distance:
            return 0, -1.0
        # 目标在车体坐标系下的方位角
        bearing = math.atan2(dy, dx) - self.odom_yaw
        bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
        if bearing > self.search_angle_threshold:
            direction = -1      # 目标在左侧
        elif bearing < -self.search_angle_threshold:
            direction = 1       # 目标在右侧
        else:
            direction = 0       # 目标基本在正前方
        return direction, dist

    # ============================================================
    # 主仲裁
    # ============================================================
    def on_timer(self):
        cmd = self.decide()
        self.pub.publish(cmd)
        self.reason_pub.publish(Int32(data=cmd.reason))
        self.last_command = cmd.command
        self.publish_hint(cmd)

    def publish_hint(self, cmd):
        """把搜索建议发布到 /search_hint。

        方案 A 契约：执行层只依据 /guide_command 的 command 字段动作，
        忽略 search_direction / regression_distance；搜索建议仅供上层
        （人工遥控 / 导航）参考，因此用独立话题物理隔离，避免执行层误用。
        """
        hint = SearchHint()
        hint.header.stamp = self.get_clock().now().to_msg()
        hint.direction = cmd.search_direction
        hint.distance = cmd.regression_distance
        hint.reason = cmd.reason
        # 只有"盲道丢失且确实找到了可信回归目标"时才算有效建议
        hint.valid = (cmd.reason == GuideCommand.REASON_BLIND_LOST
                      and cmd.regression_distance >= 0.0)
        self.hint_pub.publish(hint)

    def decide(self):
        """按优先级仲裁，输出最终指令。"""
        cmd = GuideCommand()
        cmd.source = GuideCommand.SOURCE_SUPERVISOR
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.reason = GuideCommand.REASON_NONE
        cmd.emergency = False
        cmd.ultrasonic_override = False
        cmd.search_direction = 0
        cmd.regression_distance = -1.0

        # ---- 填充传感器状态 ----
        perc_ok = self.perception_valid()
        ultra_ok = self.ultra_valid()
        odom_ok = self.odom_valid()
        cmd.perception_valid = perc_ok
        cmd.ultrasonic_valid = ultra_ok
        cmd.ultrasonic_dist = self.ultra_dist if ultra_ok else -1.0
        cmd.odom_x = self.odom_x if odom_ok else 0.0
        cmd.odom_y = self.odom_y if odom_ok else 0.0
        cmd.odom_yaw = self.odom_yaw if odom_ok else 0.0

        if perc_ok:
            p = self.perception
            cmd.lateral_error_m = p.lateral_error_m
            cmd.obstacle_near = p.obstacle_near
            cmd.obstacle_side = p.obstacle_side
            cmd.blind_path_lost = p.blind_path_lost
            cmd.confidence = p.confidence
        else:
            cmd.confidence = 0.0

        # ---- 优先级 1：感知链路失效（摄像头故障 / 节点崩溃） ----
        # 这一条让"感知节点挂了"不再等于"可以继续走"，是独立仲裁的核心价值
        if not perc_ok:
            return self._stop(cmd, GuideCommand.REASON_PERCEPTION_TIMEOUT, emergency=True)

        # ---- 优先级 2：超声波极近距离急停（最高安全优先级） ----
        if ultra_ok and self.ultra_dist < self.ultra_stop_dist:
            cmd.ultrasonic_override = True
            self.get_logger().warn(
                f'ULTRASONIC EMERGENCY STOP: {self.ultra_dist:.2f} m',
                throttle_duration_sec=1.0)
            return self._stop(cmd, GuideCommand.REASON_ULTRASONIC, emergency=True)

        # ---- 优先级 3：视觉检测到障碍物 ----
        if cmd.obstacle_near:
            return self._stop(cmd, GuideCommand.REASON_DNN_OBSTACLE)

        # ---- 优先级 4：盲道完全丢失（附带里程回归建议） ----
        if cmd.blind_path_lost:
            direction, dist = self.compute_regression()
            cmd.search_direction = direction
            cmd.regression_distance = dist
            return self._stop(cmd, GuideCommand.REASON_BLIND_LOST)

        # ---- 优先级 5：先算出期望的行走指令 ----
        err = cmd.lateral_error_m
        if abs(err) < self.dead_zone:
            desired_command = GuideCommand.MODE_FORWARD
        elif err < 0:
            desired_command = GuideCommand.MODE_LEFT
        else:
            desired_command = GuideCommand.MODE_RIGHT

        # ---- 优先级 6：超声波中距 + 视觉前进 -> 降速慢行 ----
        # 只在"打算直行"时降速；正在转向避让时保持转向动作
        if ultra_ok and self.ultra_dist < self.ultra_caution_dist \
                and desired_command == GuideCommand.MODE_FORWARD:
            cmd.ultrasonic_override = True
            desired_command = GuideCommand.MODE_CAUTION
            cmd.confidence = 0.5
            self.get_logger().info(
                f'ultrasonic caution: {self.ultra_dist:.2f} m', throttle_duration_sec=2.0)

        # ---- 优先级 7：停滞检测（里程计参与决策） ----
        # 判定基准是"期望指令是否为运动类"：若是超声波急停/障碍物导致的停车，
        # 期望指令本就是停止，不会被误判成卡住；只有"本该走却没动"才算卡住。
        if desired_command in (GuideCommand.MODE_FORWARD, GuideCommand.MODE_LEFT,
                               GuideCommand.MODE_RIGHT, GuideCommand.MODE_CAUTION):
            if odom_ok and self.is_stuck():
                self.get_logger().warn('STUCK DETECTED: odom displacement too small',
                                       throttle_duration_sec=2.0)
                return self._stop(cmd, GuideCommand.REASON_STUCK)

        cmd.command = desired_command
        cmd.confidence = 0.5 if desired_command == GuideCommand.MODE_CAUTION else 0.9
        return cmd

    def _stop(self, cmd, reason, emergency=False):
        cmd.command = GuideCommand.MODE_STOP
        cmd.reason = reason
        cmd.emergency = emergency
        cmd.confidence = 1.0 if emergency else 0.2
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
