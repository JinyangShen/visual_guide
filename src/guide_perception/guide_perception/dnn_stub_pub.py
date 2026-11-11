import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

# 尝试导入 TROS 的 ai_msgs，如果没有则退出
try:
    from ai_msgs.msg import PerceptionTargets, Target, Roi
except ImportError:
    print("警告: 当前环境没有 ai_msgs 包（x86环境），无法运行 dnn_stub_pub。")
    import sys
    sys.exit(0)


class DnnStubPub(Node):
    def __init__(self):
        super().__init__('dnn_stub_pub')
        self.pub = self.create_publisher(PerceptionTargets, '/hobot_dnn_detection', 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info('dnn_stub_pub started, publishing fake obstacle')

    def tick(self):
        msg = PerceptionTargets()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        target = Target()
        target.type = 'obstacle'
        roi = Roi()
        roi.rect.x_offset = 200
        roi.rect.y_offset = 300
        roi.rect.width = 240
        roi.rect.height = 180
        roi.confidence = 0.9
        target.rois.append(roi)
        msg.targets.append(target)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DnnStubPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
