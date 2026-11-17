"""上层应用：把 /search_hint 搜索建议转成人工可读的提示。

这是"方案 A"里"给上层的建议"通道的消费端。执行层（guide_executor）**不订阅**
/search_hint，因此这里的提示只影响人（遥控员）或导航策略，不会直接驱动车轮。

典型用法：
  - 遥控台屏幕显示"建议向左回归 1.2 米"
  - 接 TTS 播报"盲道丢失，向左搜索"
  - 导航层把回归方向作为代价地图的偏置

输入：
  /search_hint   (guide_interfaces/SearchHint)  安全仲裁节点发布的回归建议

输出：
  /search_hint_text (std_msgs/String)  中文提示文本，便于接 TTS / UI
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from guide_interfaces.msg import GuideCommand, SearchHint


class SearchHintNode(Node):
    """订阅搜索建议，产出带节流的中文提示。"""

    def __init__(self):
        """初始化参数与订阅。"""
        super().__init__('search_hint')

        # 提示节流：同一建议在该时间内不重复播报，避免 TTS 刷屏
        self.declare_parameter('throttle_sec', 5.0)
        self.throttle_sec = self.get_parameter('throttle_sec').value

        self.declare_parameter('publish_topic', '/search_hint_text')
        self.publish_topic = self.get_parameter('publish_topic').value

        self.text_pub = self.create_publisher(String, self.publish_topic, 10)
        self.create_subscription(SearchHint, '/search_hint', self.on_hint, 10)

        self.last_text = ''
        self.last_time = None
        self.hint_count = 0

        self.get_logger().info(
            f'search_hint 消费者已启动：订阅 /search_hint，'
            f'输出 {self.publish_topic}，节流 {self.throttle_sec}s')

    def build_text(self, msg):
        """把建议转成中文提示文本。"""
        if msg.direction < 0:
            side = '向左'
        elif msg.direction > 0:
            side = '向右'
        else:
            side = '直行'
        text = f'盲道丢失，建议{side}回归 {msg.distance:.1f} 米'
        if msg.reason == GuideCommand.REASON_BLIND_LOST:
            text += '（请人工确认或等待重新识别）'
        return text

    def on_hint(self, msg):
        """收到搜索建议。"""
        if not msg.valid:
            return

        text = self.build_text(msg)
        now = self.get_clock().now()

        # 节流：同一提示在 throttle_sec 内只播一次
        if self.last_time is not None and text == self.last_text:
            elapsed = (now - self.last_time).nanoseconds * 1e-9
            if elapsed < self.throttle_sec:
                return
        self.last_text = text
        self.last_time = now
        self.hint_count += 1

        self.get_logger().info(text)
        self.text_pub.publish(String(data=text))


def main(args=None):
    """节点入口。"""
    rclpy.init(args=args)
    node = SearchHintNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
