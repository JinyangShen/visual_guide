import glob
import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FakeImagePub(Node):
    def __init__(self):
        super().__init__('fake_image_pub')
        self.pub = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()

        # 默认读取 ~/guide_ws/images/*.jpg，可通过参数覆盖
        default_glob = os.path.expanduser('~/guide_ws/images/*.jpg')
        self.image_glob = self.declare_parameter('image_glob', default_glob).value
        self.files = sorted(glob.glob(self.image_glob))
        self.idx = 0

        if not self.files:
            self.get_logger().error(f'No images found: {self.image_glob}')
            return

        self.get_logger().info(f'Found {len(self.files)} images, publishing at 10 Hz')
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        if not self.files:
            return
        path = self.files[self.idx % len(self.files)]
        self.idx += 1
        img = cv2.imread(path)
        if img is None:
            self.get_logger().warn(f'Failed to read: {path}')
            return

        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')  # 必须是 bgr8
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeImagePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
