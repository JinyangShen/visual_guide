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

        # HSV 黄色阈值
        self.h_min = self.declare_parameter('h_min', 16).value
        self.h_max = self.declare_parameter('h_max', 40).value
        self.s_min = self.declare_parameter('s_min', 80).value
        self.v_min = self.declare_parameter('v_min', 80).value

        # 死区与丢失阈值
        self.dead_zone = self.declare_parameter('dead_zone', 0.15).value
        self.min_area = self.declare_parameter('min_area', 500).value
        
        # 新增：丢失容忍计数器（10Hz 下，5帧 = 0.5秒）
        self.lost_count = 0
        self.max_lost_frames = 5 

        self.get_logger().info('guide_perception_node started (FitLine + Tolerance mode)')

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
        roi = frame[int(h * 0.55):h, :]  # 只看下半部分
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower = np.array([self.h_min, self.s_min, self.v_min])
        upper = np.array([self.h_max, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cmd = GuideCommand()

        # --- 盲道丢失逻辑 ---
        if not contours:
            return self.handle_lost(cmd)

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            return self.handle_lost(cmd)

        # --- 核心升级：边界拟合 (FitLine) ---
        # 获取盲道的所有像素点
        pts = largest.reshape(-1, 2)
        # 用所有点拟合出一条直线，得到方向向量 (vx, vy) 和直线上的一点 (x0, y0)
        [vx, vy, x0, y0] = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        
        # 假设盲道一直延伸到画面最底部 (y = roi_height)，算出底部的 x 坐标
        roi_h = roi.shape[0]
        if vy == 0:
            bottom_x = x0
        else:
            bottom_x = x0 + (roi_h - y0) * (vx / vy)
            
        # 计算误差（相对于画面中心）
        err = (bottom_x - w / 2.0) / (w / 2.0)
        err = float(np.clip(err, -1.0, 1.0)) # 限制在 [-1, 1] 之间

        # 恢复正常，重置丢失计数器
        self.lost_count = 0 
        cmd.lateral_error_m = err
        cmd.blind_path_lost = False
        cmd.obstacle_near = False
        cmd.obstacle_side = 0
        cmd.confidence = 0.9

        if abs(err) < self.dead_zone:
            cmd.command = GuideCommand.MODE_FORWARD
        elif err < 0:
            cmd.command = GuideCommand.MODE_LEFT
        else:
            cmd.command = GuideCommand.MODE_RIGHT

        return cmd

    def handle_lost(self, cmd):
        """处理盲道丢失的情况，加入时间容忍度"""
        self.lost_count += 1
        
        if self.lost_count <= self.max_lost_frames:
            # 短暂丢失，输出警告（CAUTION）
            cmd.command = GuideCommand.MODE_CAUTION
            cmd.blind_path_lost = False # 主观上认为只是短暂遮挡
            cmd.confidence = 0.4
        else:
            # 长时间丢失，紧急停止
            cmd.command = GuideCommand.MODE_STOP
            cmd.blind_path_lost = True
            cmd.confidence = 0.1
            
        cmd.lateral_error_m = 0.0
        cmd.obstacle_near = False
        cmd.obstacle_side = 0
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
