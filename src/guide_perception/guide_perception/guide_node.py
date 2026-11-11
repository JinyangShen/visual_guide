import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from guide_interfaces.msg import GuideCommand


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

        # 障碍物判定参数 (画面底部中心区域非黄色的占比阈值)
        self.obstacle_area_thresh = self.declare_parameter('obstacle_area_thresh', 0.3).value

        self.get_logger().info('guide_perception_node started (Fusion mode)')

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
        roi = frame[int(h * 0.55):h, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower = np.array([self.h_min, self.s_min, self.v_min])
        upper = np.array([self.h_max, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        
        # 去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 1. 检测障碍物 (非盲道区域的密集程度)
        obstacle_near, obstacle_side = self.detect_obstacle(roi, mask, w)

        # 2. 检测盲道线
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 如果没有盲道
        if not contours:
            return self.handle_lost(obstacle_near, obstacle_side)
            
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            return self.handle_lost(obstacle_near, obstacle_side)

        # 盲道拟合
        pts = largest.reshape(-1, 2)
        [vx, vy, x0, y0] = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        roi_h = roi.shape[0]
        bottom_x = x0 if vy == 0 else x0 + (roi_h - y0) * (vx / vy)
        err = float(np.clip((bottom_x - w / 2.0) / (w / 2.0), -1.0, 1.0))

        self.lost_count = 0
        return self.fuse_decision(err, False, obstacle_near, obstacle_side)

    def detect_obstacle(self, roi, mask, img_width):
        """纯视觉启发式：看画面下半部中心区域，非黄色的像素是否太密集"""
        h, w = roi.shape[:2]
        # 取 ROI 的下半部分的中心 1/3 区域（最容易撞到东西的地方）
        check_region = roi[int(h*0.5):h, int(w*0.33):int(w*0.66)]
        check_mask = mask[int(h*0.5):h, int(w*0.33):int(w*0.66)]

        if check_region.size == 0:
            return False, 0

        # 计算该区域中“非盲道像素”的比例
        total_pixels = check_region.shape[0] * check_region.shape[1]
        non_blind_pixels = total_pixels - cv2.countNonZero(check_mask)
        ratio = non_blind_pixels / total_pixels

        # 如果非盲道像素占比超过阈值，说明前方是异物，不是路
        if ratio > self.obstacle_area_thresh:
            # 粗略判断障碍物在左还是右（基于全图非盲道重心，简单处理）
            return True, 2  # 默认中间
            
        return False, 0

    def handle_lost(self, obs_near, obs_side):
        self.lost_count += 1
        if self.lost_count <= self.max_lost_frames:
            return self.fuse_decision(0.0, True, obs_near, obs_side, is_caution=True)
        else:
            return self.fuse_decision(0.0, True, obs_near, obs_side, is_stop=True)

    def fuse_decision(self, err, lost, obs_near, obs_side, is_caution=False, is_stop=False):
        """决策融合：安全第一"""
        cmd = GuideCommand()
        cmd.lateral_error_m = err
        cmd.obstacle_near = obs_near
        cmd.obstacle_side = obs_side
        
        # 优先级 1：前方正中间有障碍物，或者彻底丢失盲道 -> 急刹
        if is_stop or obs_near:
            cmd.command = GuideCommand.MODE_STOP
            cmd.blind_path_lost = lost if is_stop else False
            cmd.confidence = 0.2
            return cmd

        # 优先级 2：盲道短暂丢失 -> 警告
        if is_caution:
            cmd.command = GuideCommand.MODE_CAUTION
            cmd.blind_path_lost = False
            cmd.confidence = 0.4
            return cmd

        # 优先级 3：正常循线
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
