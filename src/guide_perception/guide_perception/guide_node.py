import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from guide_interfaces.msg import GuideCommand


class GuideNode(Node):
    def __init__(self):
        super().__init__('guide_perception_node')
        self.create_subscription(Image, '/image_raw', self.on_image, 10)
        self.pub = self.create_publisher(GuideCommand, '/guide_command', 10)
        self.timer = self.create_timer(0.1, self.on_timer)  # 10 Hz
        self.last_image = None
        self.get_logger().info('guide_perception_node started (stub mode)')

    def on_image(self, msg):
        self.last_image = msg

    def on_timer(self):
        msg = GuideCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command = GuideCommand.MODE_FORWARD
        msg.lateral_error_m = 0.0
        msg.blind_path_lost = False
        msg.obstacle_near = False
        msg.obstacle_side = 0
        msg.confidence = 1.0
        self.pub.publish(msg)


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
