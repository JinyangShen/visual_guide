"""导盲核心决策节点（RDK X5 多传感器融合版）

输入：
  /image_raw         (sensor_msgs/Image)      视觉盲道 + 障碍物
  /hobot_dnn_detection (ai_msgs, 可选)        DNN 障碍物检测
  /ultrasonic/range  (sensor_msgs/Range)      超声波近距兜底
  /odom              (nav_msgs/Odometry)      SLAM 定位输出

输出：
  /perception_state  (guide_interfaces/PerceptionState)  感知结果（始终发布）
  /guide_command     (guide_interfaces/GuideCommand)      最终指令

两种工作模式（fusion_mode 参数）：
  inline  —— 本节点内部完成全部融合并直接发布 /guide_command（默认，单节点运行）
  external—— 本节点只负责视觉感知，发布 /perception_state，
             由独立的 safety_supervisor_node 仲裁后发布 /guide_command。
             独立仲裁保证感知进程崩溃时依然能输出停止指令。

融合优先级（从高到低）：
  1. 超声波 < 0.2 m            -> MODE_STOP（最高安全优先级）
  2. 视觉障碍 / 盲道完全丢失    -> MODE_STOP
  3. 超声波 0.2~0.5 m 且视觉前进 -> MODE_CAUTION（降速慢行）
  4. 视觉盲道偏移               -> MODE_FORWARD / LEFT / RIGHT

容错：超声波话题超时（默认 1s 无数据）时自动降级为纯视觉模式，
      视觉节点可独立运行，超声波仅作为安全增强。
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from guide_interfaces.msg import GuideCommand, PerceptionState

# 尝试导入 ai_msgs 用于 DNN 接收
try:
    from ai_msgs.msg import PerceptionTargets
    HAS_AI_MSGS = True
except ImportError:
    HAS_AI_MSGS = False


class GuideNode(Node):
    def __init__(self):
        super().__init__('guide_perception_node')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/image_raw', self.on_image, 10)
        # 感知结果：无论哪种模式都发布，供外部仲裁/记录使用
        self.perception_pub = self.create_publisher(PerceptionState, '/perception_state', 10)

        # ---- 工作模式 ----
        # inline  : 本节点直接发布 /guide_command（单节点运行）
        # external: 只发布 /perception_state，由 safety_supervisor 仲裁
        self.fusion_mode = self.declare_parameter('fusion_mode', 'inline').value
        self.publish_command = (self.fusion_mode == 'inline')
        # external 模式下完全不创建 /guide_command 的 publisher，
        # 避免下游用 "publisher 数量" 做健康判断时把本节点误算进去
        self.pub = self.create_publisher(GuideCommand, '/guide_command', 10) \
            if self.publish_command else None

        # ---- 盲道 HSV 阈值 ----
        self.h_min = self.declare_parameter('h_min', 16).value
        self.h_max = self.declare_parameter('h_max', 40).value
        self.s_min = self.declare_parameter('s_min', 80).value
        self.v_min = self.declare_parameter('v_min', 80).value

        # ---- 控制参数 ----
        self.dead_zone = self.declare_parameter('dead_zone', 0.15).value
        self.min_area = self.declare_parameter('min_area', 500).value
        self.lost_count = 0
        self.max_lost_frames = 5
        self.blind_found = False   # 本帧是否检测到盲道（供感知状态发布）
        self.obstacle_area_thresh = self.declare_parameter('obstacle_area_thresh', 0.3).value

        # ---- 超声波融合参数 ----
        # 急停阈值与减速阈值（米）
        self.ultra_stop_dist = self.declare_parameter('ultra_stop_dist', 0.2).value
        self.ultra_caution_dist = self.declare_parameter('ultra_caution_dist', 0.5).value
        # 超声波超时时间（秒）：超过则认为节点未启动，自动降级
        self.ultra_timeout = self.declare_parameter('ultra_timeout', 1.0).value
        # 超声波状态缓存
        self.ultra_dist = -1.0            # 最近一次有效距离，-1 表示无数据
        self.ultra_last_time = None       # 最近一次收到数据的时间戳

        # ---- 定位状态（来自 SLAM /odom） ----
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_received = False

        # ---- DNN 状态 ----
        self.obstacle_from_dnn = False
        self.img_w = 640
        self.img_h = 480

        # 订阅超声波与里程计
        self.create_subscription(Range, '/ultrasonic/range', self.on_ultrasonic, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        # 如果环境支持 DNN，则订阅相关话题
        if HAS_AI_MSGS:
            self.create_subscription(PerceptionTargets, '/hobot_dnn_detection', self.on_dnn, 10)
            self.get_logger().info('guide_perception_node started (DNN + HSV + Ultrasonic + Odom)')
        else:
            self.get_logger().info(
                'guide_perception_node started (HSV + Ultrasonic + Odom, no ai_msgs)')

    # ============================================================
    # 传感器回调
    # ============================================================
    def on_ultrasonic(self, msg: Range):
        """缓存超声波距离。nan/超量程视为无效数据。"""
        if math.isnan(msg.range) or msg.range < msg.min_range or msg.range > msg.max_range:
            return  # 保持旧值，等待超时机制降级
        self.ultra_dist = msg.range
        self.ultra_last_time = self.get_clock().now()

    def on_odom(self, msg: Odometry):
        """缓存 SLAM 定位（位置 + 朝向）。"""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        # 四元数 -> yaw（仅取偏航角）
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def ultra_is_valid(self):
        """判断超声波数据是否在超时窗口内（节点未启动时自动降级）。"""
        if self.ultra_last_time is None:
            return False
        age = (self.get_clock().now() - self.ultra_last_time).nanoseconds * 1e-9
        return age < self.ultra_timeout

    def on_dnn(self, msg):
        """接收 DNN 检测结果，判断正下方是否有障碍物"""
        self.obstacle_from_dnn = False
        for target in msg.targets:
            # 根据实际模型的类别名修改，如 'person', 'chair', 'obstacle'
            if target.type in ('person', 'obstacle'):
                for roi in target.rois:
                    cx = roi.rect.x_offset + roi.rect.width / 2.0
                    bottom = roi.rect.y_offset + roi.rect.height
                    # 框的中心在画面中间 1/3，且底部在画面下半部分
                    if (abs(cx - self.img_w / 2) < self.img_w / 6 and
                            bottom > self.img_h * 0.7):
                        self.obstacle_from_dnn = True
                        return

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}')
            # 视觉处理失败：发布 image_valid=false 的感知状态，
            # 由安全仲裁按"感知失效"兜底为停止指令
            self.publish_perception(msg, None)
            return

        cmd = self.compute_command(frame)
        cmd.header = msg.header

        # ---- 附加传感器状态到指令消息（供下游记录/调试） ----
        valid = self.ultra_is_valid()
        cmd.ultrasonic_valid = valid
        cmd.ultrasonic_dist = self.ultra_dist if valid else -1.0
        cmd.odom_x = self.odom_x
        cmd.odom_y = self.odom_y
        cmd.odom_yaw = self.odom_yaw
        cmd.source = GuideCommand.SOURCE_INLINE
        cmd.perception_valid = True

        # 感知结果始终发布（外部仲裁节点的输入）
        self.publish_perception(msg, cmd)

        # 仅 inline 模式由本节点直接下发指令
        if self.publish_command:
            self.pub.publish(cmd)

    def publish_perception(self, msg, cmd):
        """把视觉感知结果发布到 /perception_state，供安全仲裁节点消费。"""
        state = PerceptionState()
        state.header = msg.header
        if cmd is None:
            state.lateral_error_m = 0.0
            state.blind_path_found = False
            state.blind_path_lost = True
            state.obstacle_near = False
            state.obstacle_side = 0
            state.confidence = 0.0
            state.image_valid = False
        else:
            state.lateral_error_m = cmd.lateral_error_m
            state.blind_path_found = self.blind_found
            state.blind_path_lost = cmd.blind_path_lost
            state.obstacle_near = cmd.obstacle_near
            state.obstacle_side = cmd.obstacle_side
            state.confidence = cmd.confidence
            state.image_valid = True
        self.perception_pub.publish(state)

    # ============================================================
    # 视觉处理
    # ============================================================
    def compute_command(self, frame):
        h, w = frame.shape[:2]
        self.img_w, self.img_h = w, h  # 动态更新画面尺寸

        roi = frame[int(h * 0.55):h, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower = np.array([self.h_min, self.s_min, self.v_min])
        upper = np.array([self.h_max, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 1. 获取障碍物状态（DNN 优先，如果没有 DNN 则用 HSV 盲道启发式兜底）
        if HAS_AI_MSGS:
            obs_near = self.obstacle_from_dnn
            obs_side = 2 if obs_near else 0
        else:
            obs_near, obs_side = self.detect_obstacle_hsv(roi, mask, w)

        # 2. 检测盲道线
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.blind_found = False
            return self.handle_lost(obs_near, obs_side)

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            self.blind_found = False
            return self.handle_lost(obs_near, obs_side)

        # 盲道拟合
        pts = largest.reshape(-1, 2)
        [vx, vy, x0, y0] = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        roi_h = roi.shape[0]
        bottom_x = x0 if vy == 0 else x0 + (roi_h - y0) * (vx / vy)
        err = float(np.clip((bottom_x - w / 2.0) / (w / 2.0), -1.0, 1.0))

        self.lost_count = 0
        self.blind_found = True
        return self.fuse_decision(err, False, obs_near, obs_side)

    def detect_obstacle_hsv(self, roi, mask, img_width):
        """纯视觉启发式（用于没有 DNN 的 x86 环境）"""
        h, w = roi.shape[:2]
        check_region = roi[int(h*0.5):h, int(w*0.33):int(w*0.66)]
        check_mask = mask[int(h*0.5):h, int(w*0.33):int(w*0.66)]

        if check_region.size == 0:
            return False, 0

        total_pixels = check_region.shape[0] * check_region.shape[1]
        non_blind_pixels = total_pixels - cv2.countNonZero(check_mask)
        ratio = non_blind_pixels / total_pixels

        if ratio > self.obstacle_area_thresh:
            return True, 2
        return False, 0

    def handle_lost(self, obs_near, obs_side):
        self.lost_count += 1
        if self.lost_count <= self.max_lost_frames:
            return self.fuse_decision(0.0, True, obs_near, obs_side, is_caution=True)
        else:
            return self.fuse_decision(0.0, True, obs_near, obs_side, is_stop=True)

    # ============================================================
    # 决策级融合（核心）
    # ============================================================
    def fuse_decision(self, err, lost, obs_near, obs_side, is_caution=False, is_stop=False):
        """按优先级融合超声波、视觉、定位数据，输出最终指令。

        注意：这是 inline 模式下的内置融合，安全仲裁请使用独立的
        safety_supervisor_node（external 模式），后者额外具备感知失效兜底、
        停滞检测与盲道丢失回归方向计算。
        """
        cmd = GuideCommand()
        cmd.lateral_error_m = err
        cmd.obstacle_near = obs_near
        cmd.obstacle_side = obs_side
        cmd.ultrasonic_override = False
        cmd.reason = GuideCommand.REASON_NONE
        cmd.emergency = False
        cmd.source = GuideCommand.SOURCE_INLINE

        ultra_valid = self.ultra_is_valid()
        ultra_dist = self.ultra_dist if ultra_valid else float('inf')

        # ---- 优先级 1：超声波极近距离急停（最高安全优先级） ----
        # 无论视觉结果如何，盲区障碍必须立即停车
        if ultra_valid and ultra_dist < self.ultra_stop_dist:
            cmd.command = GuideCommand.MODE_STOP
            cmd.ultrasonic_override = True
            cmd.confidence = 1.0
            cmd.blind_path_lost = False
            cmd.reason = GuideCommand.REASON_ULTRASONIC
            cmd.emergency = True
            self.get_logger().warn(
                f'ULTRASONIC EMERGENCY STOP: {ultra_dist:.2f} m', throttle_duration_sec=1.0)
            return cmd

        # ---- 优先级 2：视觉障碍物 或 盲道完全丢失 ----
        if is_stop or obs_near:
            cmd.command = GuideCommand.MODE_STOP
            cmd.blind_path_lost = lost if is_stop else False
            cmd.confidence = 0.2
            cmd.reason = GuideCommand.REASON_BLIND_LOST if is_stop \
                else GuideCommand.REASON_DNN_OBSTACLE
            return cmd

        # ---- 优先级 3：先算出视觉循迹结果 ----
        if abs(err) < self.dead_zone:
            visual_command = GuideCommand.MODE_FORWARD
        elif err < 0:
            visual_command = GuideCommand.MODE_LEFT
        else:
            visual_command = GuideCommand.MODE_RIGHT

        # ---- 优先级 4：超声波中距预警 + 视觉前进 -> 降速慢行 ----
        # 只在"打算直行"时降速；正在转向避让时保持转向动作
        if ultra_valid and ultra_dist < self.ultra_caution_dist \
                and visual_command == GuideCommand.MODE_FORWARD:
            cmd.command = GuideCommand.MODE_CAUTION
            cmd.ultrasonic_override = True
            cmd.confidence = 0.5
            cmd.blind_path_lost = False
            self.get_logger().info(
                f'ultrasonic caution: {ultra_dist:.2f} m', throttle_duration_sec=2.0)
            return cmd

        # 盲道短暂丢失的惯性慢行
        if is_caution:
            cmd.command = GuideCommand.MODE_CAUTION
            cmd.blind_path_lost = False
            cmd.confidence = 0.4
            return cmd

        # ---- 优先级 5：正常循迹 ----
        cmd.command = visual_command
        cmd.blind_path_lost = False
        cmd.confidence = 0.9
        return cmd


def main(args=None):
    rclpy.init(args=args)
    node = GuideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
