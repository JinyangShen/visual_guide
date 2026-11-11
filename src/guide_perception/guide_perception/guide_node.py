import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from guide_interfaces.msg import GuideCommand

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
        self.pub = self.create_publisher(GuideCommand, '/guide_command', 10)

        # 盲道 HSV 阈值
        self.h_min = self.declare_parameter('h_min', 16).value
        self.h_max = self.declare_parameter('h_max', 40).value
        self.s_min = self.declare_parameter('s_min', 80).value
        self.v_min = self.declare_parameter('v_min', 80).value

        # 控制参数
        self.dead_zone = self.declare_parameter('dead_zone', 0.15).value
        self.min_area = self.declare_parameter('min_area', 500).value
        self.lost_count = 0
        self.max_lost_frames = 5 
        self.obstacle_area_thresh = self.declare_parameter('obstacle_area_thresh', 0.3).value

        # DNN 状态
        self.obstacle_from_dnn = False
        self.img_w = 640
        self.img_h = 480

        # 如果环境支持 DNN，则订阅相关话题
        if HAS_AI_MSGS:
            self.create_subscription(PerceptionTargets, '/hobot_dnn_detection', self.on_dnn, 10)
            self.get_logger().info('guide_perception_node started (DNN + HSV mode)')
        else:
            self.get_logger().info('guide_perception_node started (HSV only mode, no ai_msgs)')

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
            return

        cmd = self.compute_command(frame)
        cmd.header = msg.header
        self.pub.publish(cmd)

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
            return self.handle_lost(obs_near, obs_side)
            
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            return self.handle_lost(obs_near, obs_side)

        # 盲道拟合
        pts = largest.reshape(-1, 2)
        [vx, vy, x0, y0] = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        roi_h = roi.shape[0]
        bottom_x = x0 if vy == 0 else x0 + (roi_h - y0) * (vx / vy)
        err = float(np.clip((bottom_x - w / 2.0) / (w / 2.0), -1.0, 1.0))

        self.lost_count = 0
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

    def fuse_decision(self, err, lost, obs_near, obs_side, is_caution=False, is_stop=False):
        cmd = GuideCommand()
        cmd.lateral_error_m = err
        cmd.obstacle_near = obs_near
        cmd.obstacle_side = obs_side
        
        if is_stop or obs_near:
            cmd.command = GuideCommand.MODE_STOP
            cmd.blind_path_lost = lost if is_stop else False
            cmd.confidence = 0.2
            return cmd

        if is_caution:
            cmd.command = GuideCommand.MODE_CAUTION
            cmd.blind_path_lost = False
            cmd.confidence = 0.4
            return cmd

        cmd.blind_path_lost = False
        cmd.confidence = 0.9
        if abs(err) < self.dead_zone:
            cmd.command = GuideCommand.MODE_FORWARD
        elif err < 0:
            cmd.command = GuideCommand.MODE_LEFT
        else:
            cmd.command = GuideCommand.MODE_RIGHT

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
